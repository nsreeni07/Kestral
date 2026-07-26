// main.cpp
// Intentionally thin, per project requirements: initialize hardware,
// initialize drivers, initialize state machine, initialize logger,
// start scheduler, run main loop. All actual logic lives in the task
// functions below, which delegate to the relevant module.
//
// NOTE: HAL peripheral handles (hspi1, hi2c1, htim1, etc.) are declared
// extern here as placeholders. In a real CubeIDE/PlatformIO project
// these come from the CubeMX-generated main.c initialization
// (MX_SPI1_Init(), MX_I2C1_Init(), MX_TIM1_Init(), ...) — call those
// first in main() before constructing the driver objects below, since
// each driver constructor just stores the handle pointer and expects
// the peripheral to already be configured.

#include "config.h"
#include "math_utils.h"
#include "flight_data.h"
#include "sensors.h"
#include "peripherals.h"
#include "ahrs.h"
#include "state_machine.h"
#include "tvc_control.h"
#include "flight_logger.h"
#include "scheduler.h"

// ---- Placeholder externs for CubeMX-generated peripheral handles -------
// Replace with #include "main.h" from your CubeMX project, which
// declares the real extern handles (hspi1, hi2c1, htim1, hadc1, hsd1...).
extern SPI_HandleTypeDef hspi1;   // ICM-45686
extern I2C_HandleTypeDef hi2c1;   // BMP580
extern I2C_HandleTypeDef hi2c2;   // BQ25883 charger
extern TIM_HandleTypeDef htim1;   // servo PWM (pitch/yaw channels)
extern TIM_HandleTypeDef htim2;   // WS2812 status LED
extern ADC_HandleTypeDef hadc1;   // battery voltage
extern SD_HandleTypeDef hsd1;     // microSD

extern "C" void SystemClock_Config();
extern "C" void MX_GPIO_Init();
extern "C" void MX_SPI1_Init();
extern "C" void MX_I2C1_Init();
extern "C" void MX_I2C2_Init();
extern "C" void MX_TIM1_Init();
extern "C" void MX_TIM2_Init();
extern "C" void MX_ADC1_Init();
extern "C" void MX_SDIO_Init();
extern "C" void MX_TIM6_Init(); // base scheduler tick timer
extern "C" void HAL_Init();
extern "C" void HAL_IWDG_Refresh(void*);
// -------------------------------------------------------------------------

namespace {

// ---- Module instances (file-scope so task functions can reach them) ----
Sensors::ICM45686        g_imu(&hspi1, Config::Pins::IMU_CS_PIN);
Sensors::BMP580           g_baro(&hi2c1, Config::Pins::BARO_ADDR);

Peripherals::ServoDriver  g_pitch_servo(&htim1, Config::Pins::SERVO_PITCH_CHANNEL);
Peripherals::ServoDriver  g_yaw_servo(&htim1, Config::Pins::SERVO_YAW_CHANNEL);
Peripherals::StatusLED    g_led(&htim2, 0);
Peripherals::SDCardIO     g_sd(&hsd1);
Peripherals::BatteryMonitor g_battery(&hadc1, Config::Pins::BATTERY_ADC_CHANNEL,
                                       Config::Power::ADC_DIVIDER_RATIO,
                                       Config::Power::ADC_VREF,
                                       Config::Power::ADC_MAX_COUNTS);
Peripherals::ChargerMonitor g_charger(&hi2c2, 0x6B); // BQ25883 default 7-bit addr

AHRS::MadgwickFilter      g_ahrs(Config::AHRS::GYRO_SAMPLE_DT);
AHRS::AltitudeEstimator   g_altitude(Config::Baro::BARO_SAMPLE_DT, Config::Baro::VELOCITY_LPF_ALPHA);

Flight::FlightStateMachine g_state_machine;
Control::TVCController     g_tvc(g_pitch_servo, g_yaw_servo);
Logging::FlightLogger       g_logger(g_sd, Config::Logging::BUFFER_SIZE_BYTES);
Utilities::FaultManager     g_fault_manager(g_led);

Core::Scheduler g_scheduler;

Flight::FlightData g_flight_data; // the shared snapshot

Flight::State g_last_logged_state = Flight::State::IDLE;

// ---------------------------------------------------------------------
// Task: IMU read (1000 Hz) — reads accel/gyro, runs Madgwick update.
// ---------------------------------------------------------------------
void taskImuRead() {
    MathUtils::Vector3 accel, gyro;

    bool accel_ok = g_imu.readAcceleration(accel);
    bool gyro_ok = g_imu.readGyroscope(gyro);

    if (!accel_ok || !gyro_ok) {
        g_fault_manager.raiseFault(Flight::FaultCode::IMU_COMM_TIMEOUT, /*critical=*/true);
        g_state_machine.forceFault();
        return;
    }

    g_flight_data.accel_mps2 = accel;
    g_flight_data.gyro_dps = gyro;

    // Beta schedule based on current flight state (see config.h comment
    // on AHRS gain scheduling rationale).
    switch (g_state_machine.currentState()) {
        case Flight::State::IDLE:
        case Flight::State::ARMED:
            g_ahrs.setBeta(Config::AHRS::BETA_GROUND);
            break;
        case Flight::State::LAUNCH_DETECTED:
        case Flight::State::POWERED_ASCENT:
            g_ahrs.setBeta(Config::AHRS::BETA_POWERED);
            break;
        case Flight::State::COAST:
        case Flight::State::APOGEE:
            g_ahrs.setBeta(Config::AHRS::BETA_COAST);
            break;
        case Flight::State::DESCENT:
        case Flight::State::LANDED:
            g_ahrs.setBeta(Config::AHRS::BETA_DESCENT);
            break;
        default:
            break;
    }

    g_ahrs.update(gyro, accel);
    g_ahrs.getEulerDeg(g_flight_data.roll_deg, g_flight_data.pitch_deg, g_flight_data.yaw_deg);
}

// ---------------------------------------------------------------------
// Task: barometer read (50 Hz) — pressure, temperature, altitude,
// vertical velocity.
// ---------------------------------------------------------------------
void taskBaroRead() {
    float pressure_hpa = 0.0f;
    float temp_c = 0.0f;

    bool p_ok = g_baro.readPressure(pressure_hpa);
    bool t_ok = g_baro.readTemperature(temp_c);

    if (!p_ok || !t_ok) {
        g_fault_manager.raiseFault(Flight::FaultCode::BARO_COMM_TIMEOUT, /*critical=*/true);
        g_state_machine.forceFault();
        return;
    }

    g_flight_data.pressure_hpa = pressure_hpa;
    g_flight_data.temperature_c = temp_c;

    g_altitude.update(pressure_hpa);
    g_flight_data.altitude_m = g_altitude.altitudeM();
    g_flight_data.vertical_velocity_mps = g_altitude.verticalVelocityMps();
}

// ---------------------------------------------------------------------
// Task: control loop (500 Hz) — state machine update + TVC PID.
// ---------------------------------------------------------------------
void taskControlLoop() {
    uint32_t now_ms = g_scheduler.nowMs();
    float dt = 1.0f / Config::Rates::CONTROL_HZ;

    float accel_mag = g_flight_data.accel_mps2.magnitude();

    g_state_machine.update(now_ms, accel_mag,
                            g_flight_data.altitude_m, g_flight_data.vertical_velocity_mps);
    g_flight_data.flight_state = g_state_machine.currentState();

    // On first entry to ARMED (from a ground command elsewhere — arm()
    // is called externally, e.g. via a debug UART command or GPIO
    // switch, not shown here), capture ground pressure reference.
    if (g_flight_data.flight_state == Flight::State::ARMED &&
        g_last_logged_state != Flight::State::ARMED) {
        g_flight_data.ground_pressure_hpa = g_flight_data.pressure_hpa;
        g_altitude.setGroundPressure(g_flight_data.ground_pressure_hpa);
    }
    g_last_logged_state = g_flight_data.flight_state;

    g_tvc.update(g_flight_data.pitch_deg, g_flight_data.yaw_deg, dt,
                 g_state_machine.tvcAuthorized(),
                 g_flight_data.pid_output_pitch, g_flight_data.pid_output_yaw,
                 g_flight_data.servo_pitch_deg, g_flight_data.servo_yaw_deg);
}

// ---------------------------------------------------------------------
// Task: logging (100 Hz) — snapshot + buffered CSV write.
// ---------------------------------------------------------------------
void taskLogData() {
    g_flight_data.timestamp_ms = g_scheduler.nowMs();
    g_flight_data.battery_voltage = g_battery.readVoltage();
    g_flight_data.active_faults = g_fault_manager.activeFaults();

    if (g_flight_data.battery_voltage <= Config::Power::BATTERY_CRITICAL_VOLTAGE) {
        g_fault_manager.raiseFault(Flight::FaultCode::BATTERY_CRITICAL, /*critical=*/false);
    }

    if (!g_logger.logRow(g_flight_data)) {
        g_fault_manager.raiseFault(Flight::FaultCode::SD_WRITE_FAIL, /*critical=*/false);
    }

    // Flush on any transition into a terminal-ish state so data isn't
    // stranded in RAM if the vehicle is on the ground/recovered soon.
    if (g_flight_data.flight_state == Flight::State::LANDED ||
        g_flight_data.flight_state == Flight::State::FAULT) {
        g_logger.flush();
    }
}

// ---------------------------------------------------------------------
// Task: status update (10 Hz) — LED pattern, watchdog refresh.
// ---------------------------------------------------------------------
void taskStatusUpdate() {
    uint32_t now_ms = g_scheduler.nowMs();

    if (!g_fault_manager.hasCriticalFault()) {
        // Solid color reflects flight state when no fault is latched.
        switch (g_flight_data.flight_state) {
            case Flight::State::IDLE:
                g_led.setColor(Peripherals::LedColor::OFF);
                break;
            case Flight::State::ARMED:
                g_led.setColor(Peripherals::LedColor::BLUE);
                break;
            case Flight::State::LANDED:
                g_led.setColor(Peripherals::LedColor::GREEN);
                break;
            default:
                g_led.setColor(Peripherals::LedColor::PURPLE); // actively flying/logging
                break;
        }
    }

    g_fault_manager.update(now_ms); // drives blink pattern if a fault is latched

    // Feed the independent watchdog — if any task hangs and this task
    // stops running, IWDG will reset the MCU. Real IWDG timeout should
    // be configured (via MX_IWDG_Init) to Config::Watchdog::TIMEOUT_MS.
    HAL_IWDG_Refresh(nullptr);
}

} // namespace

// ---------------------------------------------------------------------
// Base timer ISR — call g_scheduler.onTimerTick() from here.
// Wire this into TIM6_IRQHandler (or whichever timer you configure at
// Config::Rates::IMU_HZ, i.e. 1000Hz / 1ms period) in stm32f7xx_it.c:
//
//   extern "C" void TIM6_DAC_IRQHandler(void) {
//       HAL_TIM_IRQHandler(&htim6);
//   }
//   // and register this as the period-elapsed callback:
//   extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
//       if (htim->Instance == TIM6) { SchedulerTick(); }
//   }
// ---------------------------------------------------------------------
extern "C" void SchedulerTick() {
    g_scheduler.onTimerTick();
}

int main() {
    // ---- Hardware init ----
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_ADC1_Init();
    MX_SDIO_Init();
    MX_TIM6_Init(); // 1ms base tick for scheduler

    // ---- Driver init ----
    bool imu_ok = g_imu.init();
    bool baro_ok = g_baro.init();
    bool sd_ok = g_sd.init();

    g_pitch_servo.init(Config::Servo::PWM_MIN_US, Config::Servo::PWM_CENTER_US, Config::Servo::PWM_MAX_US);
    g_yaw_servo.init(Config::Servo::PWM_MIN_US, Config::Servo::PWM_CENTER_US, Config::Servo::PWM_MAX_US);
    g_led.init();
    g_charger.init();

    if (!imu_ok) {
        g_fault_manager.raiseFault(Flight::FaultCode::IMU_INIT_FAIL, /*critical=*/true);
    }
    if (!baro_ok) {
        g_fault_manager.raiseFault(Flight::FaultCode::BARO_INIT_FAIL, /*critical=*/true);
    }
    if (!sd_ok) {
        g_fault_manager.raiseFault(Flight::FaultCode::SD_INIT_FAIL, /*critical=*/true);
    }

    if (g_fault_manager.hasCriticalFault()) {
        g_state_machine.forceFault();
        // Stay in FAULT — do not proceed to arm/fly with a failed
        // critical sensor or logger. LED will indicate the fault code;
        // ground crew must power-cycle after fixing the issue.
    }

    // ---- Logger init ----
    if (!g_logger.begin()) {
        g_fault_manager.raiseFault(Flight::FaultCode::SD_INIT_FAIL, /*critical=*/true);
        g_state_machine.forceFault();
    }

    // ---- State machine init ----
    // g_state_machine starts in IDLE by default. Arming is triggered
    // externally (ground station command / arming switch GPIO), not
    // shown here — call g_state_machine.arm() from that handler.

    // ---- Scheduler task registration ----
    // Period in base ticks (1ms each), derived from Config::Rates.
    g_scheduler.registerTask(taskImuRead,      1000 / Config::Rates::IMU_HZ);      // every 1 tick
    g_scheduler.registerTask(taskBaroRead,     1000 / Config::Rates::BARO_HZ);     // every 20 ticks
    g_scheduler.registerTask(taskControlLoop,  1000 / Config::Rates::CONTROL_HZ);  // every 2 ticks
    g_scheduler.registerTask(taskLogData,      1000 / Config::Rates::LOG_HZ);      // every 10 ticks
    g_scheduler.registerTask(taskStatusUpdate, 1000 / Config::Rates::STATUS_HZ);   // every 100 ticks

    // ---- Main loop ----
    // Per project requirement: main contains no logic beyond this —
    // all real work happens in the registered task functions above.
    while (true) {
        g_scheduler.run();
    }
}
