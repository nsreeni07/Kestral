// sim_flight.cpp
// Native (PC) simulation harness. Compiles with plain g++, no STM32
// toolchain or hardware needed, because the HAL calls in peripherals.cpp
// are already stubbed placeholders. This links the REAL production
// logic — MadgwickFilter, AltitudeEstimator, FlightStateMachine,
// TVCController, PIDController — against a synthetic sensor feed
// generated from a simple rocket flight model, so you can validate
// control logic and tune gains before anything touches real hardware.
//
// Build (from the project root):
//   g++ -std=c++17 -Iinclude src/ahrs.cpp src/state_machine.cpp \
//       src/tvc_control.cpp src/peripherals.cpp sim/sim_flight.cpp \
//       -o sim_flight
//   ./sim_flight > flight_log.csv
//
// What the synthetic flight does:
//   0.0s - 0.3s   : on the pad (ARMED), sitting at 1g, small noise
//   0.3s - 2.0s   : POWERED_ASCENT, ~6g net accel, with an injected
//                   pitch/yaw disturbance (simulating tip-off / wind)
//                   that the TVC loop must null out
//   2.0s - 8.0s   : COAST, decelerating under gravity + drag
//   ~8.0s          : APOGEE
//   8.0s - 14s    : DESCENT under drogue/main (simple constant descent rate)
//   14s+           : LANDED

#include <cstdio>
#include <cmath>
#include "config.h"
#include "math_utils.h"
#include "flight_data.h"
#include "ahrs.h"
#include "state_machine.h"
#include "tvc_control.h"
#include "peripherals.h"

using namespace MathUtils;

int main() {
    // Dummy peripheral handles — the placeholder HAL layer never
    // dereferences these, it just no-ops, so nullptr is safe here.
    Peripherals::ServoDriver pitch_servo(nullptr, Config::Pins::SERVO_PITCH_CHANNEL);
    Peripherals::ServoDriver yaw_servo(nullptr, Config::Pins::SERVO_YAW_CHANNEL);
    pitch_servo.init(Config::Servo::PWM_MIN_US, Config::Servo::PWM_CENTER_US, Config::Servo::PWM_MAX_US);
    yaw_servo.init(Config::Servo::PWM_MIN_US, Config::Servo::PWM_CENTER_US, Config::Servo::PWM_MAX_US);

    AHRS::MadgwickFilter ahrs(Config::AHRS::GYRO_SAMPLE_DT);
    AHRS::AltitudeEstimator altitude(Config::Baro::BARO_SAMPLE_DT, Config::Baro::VELOCITY_LPF_ALPHA);
    Flight::FlightStateMachine state_machine;
    Control::TVCController tvc(pitch_servo, yaw_servo);

    state_machine.arm(); // simulate ground station arm command at t=0

    constexpr float DT = 1.0f / Config::Rates::CONTROL_HZ; // 500 Hz control tick
    constexpr float SIM_DURATION_S = 16.0f;
    constexpr int TOTAL_TICKS = static_cast<int>(SIM_DURATION_S / DT);

    // Ground-truth flight model state (what the "real rocket" is doing;
    // sensors below are derived from this with added noise/disturbance).
    float true_altitude_m = 0.0f;
    float true_velocity_mps = 0.0f;
    float ground_pressure_hpa = Config::Baro::SEA_LEVEL_PRESSURE_HPA;
    altitude.setGroundPressure(ground_pressure_hpa);

    uint32_t now_ms = 0;

    // CSV header
    std::printf("t_s,state,true_alt_m,true_vel_mps,est_alt_m,est_vel_mps,"
                "accel_mag,roll_deg,pitch_deg,yaw_deg,"
                "pid_pitch,pid_yaw,servo_pitch_deg,servo_yaw_deg\n");

    for (int tick = 0; tick < TOTAL_TICKS; ++tick) {
        float t = tick * DT;
        now_ms = static_cast<uint32_t>(t * 1000.0f);

        // ---- Ground-truth flight model ----
        Vector3 true_accel_mps2(0, 0, 9.80665f); // resting: 1g on Z (body up)
        float injected_pitch_disturbance_deg = 0.0f;
        float injected_yaw_disturbance_deg = 0.0f;

        if (t < 0.3f) {
            // On the pad: 1g, negligible motion
            true_velocity_mps = 0.0f;
        } else if (t < 2.0f) {
            // Powered ascent: ~6g net, with a decaying tip-off disturbance
            // for TVC to correct (simulates wind/rail-exit perturbation)
            true_accel_mps2 = Vector3(0, 0, 6.0f * 9.80665f);
            float phase = (t - 0.3f);
            injected_pitch_disturbance_deg = 8.0f * std::exp(-phase * 1.5f) * std::sin(phase * 6.0f);
            injected_yaw_disturbance_deg = 5.0f * std::exp(-phase * 1.5f) * std::cos(phase * 4.0f);
            true_velocity_mps += (6.0f * 9.80665f - 9.80665f) * DT;
        } else if (t < 8.0f) {
            // Coast: decelerating under gravity + light drag
            float drag = 0.02f * true_velocity_mps * true_velocity_mps;
            true_accel_mps2 = Vector3(0, 0, -(9.80665f + drag) * 0.1f); // small residual sensor reading near 0g net
            true_velocity_mps -= (9.80665f + drag) * DT;
        } else if (t < 14.0f) {
            // Descent under canopy: roughly constant descent rate
            true_velocity_mps = -6.0f; // m/s (down)
            true_accel_mps2 = Vector3(0, 0, 9.80665f); // canopy load ~1g net
        } else {
            // Landed
            true_velocity_mps = 0.0f;
            true_accel_mps2 = Vector3(0, 0, 9.80665f);
        }

        true_altitude_m += true_velocity_mps * DT;
        if (true_altitude_m < 0.0f) true_altitude_m = 0.0f;

        // ---- Derive synthetic sensor readings from ground truth ----
        Vector3 gyro_dps(
            injected_pitch_disturbance_deg > 0 ? 0.0f : 0.0f, // gyro derived via disturbance rate below
            0.0f, 0.0f);
        // Approximate gyro as derivative-ish of the injected disturbance
        // for a plausible (not exact) angular rate signal.
        static float prev_pitch_dist = 0.0f, prev_yaw_dist = 0.0f;
        float gyro_pitch_dps = (injected_pitch_disturbance_deg - prev_pitch_dist) / DT;
        float gyro_yaw_dps = (injected_yaw_disturbance_deg - prev_yaw_dist) / DT;
        prev_pitch_dist = injected_pitch_disturbance_deg;
        prev_yaw_dist = injected_yaw_disturbance_deg;
        gyro_dps = Vector3(gyro_pitch_dps, gyro_yaw_dps, 0.0f);

        // Tilt the "gravity" direction in the accel reading to encode
        // the disturbance angle, so Madgwick actually estimates a
        // nonzero pitch/yaw for the PID loop to act on.
        float pitch_rad = degToRad(injected_pitch_disturbance_deg);
        float yaw_rad = degToRad(injected_yaw_disturbance_deg);
        Vector3 accel_mps2(
            true_accel_mps2.z * std::sin(pitch_rad),
            true_accel_mps2.z * std::sin(yaw_rad),
            true_accel_mps2.z * std::cos(pitch_rad) * std::cos(yaw_rad)
        );

        float pressure_hpa = ground_pressure_hpa * std::pow(1.0f - (true_altitude_m / 44330.0f), 5.25588f);

        // ---- Feed the REAL production modules ----
        // AHRS beta schedule (mirrors main.cpp logic)
        switch (state_machine.currentState()) {
            case Flight::State::IDLE:
            case Flight::State::ARMED:
                ahrs.setBeta(Config::AHRS::BETA_GROUND); break;
            case Flight::State::LAUNCH_DETECTED:
            case Flight::State::POWERED_ASCENT:
                ahrs.setBeta(Config::AHRS::BETA_POWERED); break;
            case Flight::State::COAST:
            case Flight::State::APOGEE:
                ahrs.setBeta(Config::AHRS::BETA_COAST); break;
            default:
                ahrs.setBeta(Config::AHRS::BETA_DESCENT); break;
        }
        ahrs.update(gyro_dps, accel_mps2);

        float roll_deg, pitch_deg, yaw_deg;
        ahrs.getEulerDeg(roll_deg, pitch_deg, yaw_deg);

        altitude.update(pressure_hpa);

        float accel_mag = accel_mps2.magnitude();
        state_machine.update(now_ms, accel_mag, altitude.altitudeM(), altitude.verticalVelocityMps());

        // Capture ground pressure at ARM (t=0 here since we arm immediately)
        if (tick == 0) {
            altitude.setGroundPressure(pressure_hpa);
        }

        float pid_pitch = 0.0f, pid_yaw = 0.0f, servo_pitch = 0.0f, servo_yaw = 0.0f;
        tvc.update(pitch_deg, yaw_deg, DT, state_machine.tvcAuthorized(),
                   pid_pitch, pid_yaw, servo_pitch, servo_yaw);

        // ---- Log this tick ----
        std::printf("%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f\n",
                     t, Flight::stateToString(state_machine.currentState()),
                     true_altitude_m, true_velocity_mps,
                     altitude.altitudeM(), altitude.verticalVelocityMps(),
                     accel_mag, roll_deg, pitch_deg, yaw_deg,
                     pid_pitch, pid_yaw, servo_pitch, servo_yaw);
    }

    return 0;
}
