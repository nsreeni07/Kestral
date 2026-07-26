// peripherals.h
// "Dumb" hardware I/O wrappers with no control logic: servo PWM output,
// WS2812 status LED, SD card block interface, battery voltage ADC,
// BQ25883 charger status. Each class only translates between physical
// units and register/peripheral calls.

#pragma once

#include <cstdint>
#include <cstddef>

struct TIM_HandleTypeDef;
struct ADC_HandleTypeDef;
struct I2C_HandleTypeDef;
struct SD_HandleTypeDef;

namespace Peripherals {

// ---------------------------------------------------------------------
// ServoDriver — drives one RC-style servo via hardware timer PWM
// ---------------------------------------------------------------------
class ServoDriver {
public:
    ServoDriver(TIM_HandleTypeDef* htim, uint32_t channel);

    // Configures min/center/max pulse widths (microseconds) and starts
    // the PWM channel.
    void init(uint16_t pwm_min_us, uint16_t pwm_center_us, uint16_t pwm_max_us);

    // Commands a deflection in degrees, internally mapped to pulse width.
    // Caller is responsible for rate limiting / smoothing before this
    // call (see tvc_control.h ServoMixer) — this class just outputs.
    void setDeflectionDeg(float deg, float min_deg, float max_deg);

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    uint16_t pwm_min_us_ = 1000;
    uint16_t pwm_center_us_ = 1500;
    uint16_t pwm_max_us_ = 2000;

    void setPulseWidthUs(uint16_t us);
};

// ---------------------------------------------------------------------
// StatusLED — single WS2812 driven via TIM PWM + DMA
// ---------------------------------------------------------------------
enum class LedColor : uint8_t {
    OFF = 0,
    GREEN,   // nominal
    YELLOW,  // warning (e.g. low battery)
    RED,     // fault
    BLUE,    // armed
    PURPLE   // logging active / recording
};

class StatusLED {
public:
    explicit StatusLED(TIM_HandleTypeDef* htim, uint32_t channel);

    void init();
    void setColor(LedColor color);
    // Call periodically (e.g. at STATUS_HZ) to drive blink patterns
    // for fault indication (see fault_manager blink codes).
    void update(uint32_t now_ms);

    // Sets a repeating blink pattern: `blink_count` short flashes of
    // `color`, then a pause, repeating. Used to encode fault codes.
    void setBlinkPattern(LedColor color, uint8_t blink_count);

private:
    TIM_HandleTypeDef* htim_;
    uint32_t channel_;
    LedColor solid_color_ = LedColor::OFF;

    LedColor blink_color_ = LedColor::OFF;
    uint8_t blink_count_ = 0;
    uint8_t blink_state_ = 0;
    uint32_t last_transition_ms_ = 0;
    bool blink_active_ = false;

    void writeRGB(uint8_t r, uint8_t g, uint8_t b);
};

// ---------------------------------------------------------------------
// SDCardIO — low-level block read/write wrapper (FatFs sits on top of
// this in flight_logger.cpp; this class only wraps HAL_SD_ calls and
// exposes init/status for fault detection).
// ---------------------------------------------------------------------
class SDCardIO {
public:
    explicit SDCardIO(SD_HandleTypeDef* hsd);

    bool init();
    bool isReady() const { return ready_; }

private:
    SD_HandleTypeDef* hsd_;
    bool ready_ = false;
};

// ---------------------------------------------------------------------
// BatteryMonitor — reads pack voltage via ADC + resistor divider
// ---------------------------------------------------------------------
class BatteryMonitor {
public:
    BatteryMonitor(ADC_HandleTypeDef* hadc, uint32_t channel,
                    float divider_ratio, float vref, uint16_t adc_max_counts);

    // Blocking single conversion read, returns pack voltage in volts.
    float readVoltage();

private:
    ADC_HandleTypeDef* hadc_;
    uint32_t channel_;
    float divider_ratio_;
    float vref_;
    uint16_t adc_max_counts_;
};

// ---------------------------------------------------------------------
// ChargerMonitor — BQ25883 status/fault readback over I2C
// ---------------------------------------------------------------------
enum class ChargerState : uint8_t {
    UNKNOWN = 0,
    NOT_CHARGING,
    PRE_CHARGE,
    FAST_CHARGE,
    CHARGE_DONE,
    FAULT
};

class ChargerMonitor {
public:
    explicit ChargerMonitor(I2C_HandleTypeDef* hi2c, uint8_t device_addr);

    bool init();
    ChargerState readState();

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t device_addr_;

    bool readRegister(uint8_t reg, uint8_t& out_value);
};

} // namespace Peripherals
