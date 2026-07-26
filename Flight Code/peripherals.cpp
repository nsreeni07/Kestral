// peripherals.cpp
// Implementation of servo PWM, WS2812 status LED, SD card init wrapper,
// battery ADC monitor, and BQ25883 charger monitor.
//
// As with sensors.cpp, HAL calls below are placeholders. Swap in real
// "stm32f7xx_hal.h" calls and remove the placeholder block once wired
// to your CubeMX project.

#include "peripherals.h"
#include "math_utils.h"
#include <algorithm>

#ifndef HAL_PLACEHOLDERS_DEFINED
#define HAL_PLACEHOLDERS_DEFINED
struct TIM_HandleTypeDef {};
struct ADC_HandleTypeDef {};
struct I2C_HandleTypeDef {};
struct SD_HandleTypeDef {};
enum HAL_StatusTypeDef { HAL_OK = 0, HAL_ERROR = 1, HAL_TIMEOUT = 2 };
static inline void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef*, uint32_t, uint32_t) {}
static inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef*) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef*, uint32_t) { return HAL_OK; }
static inline uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*) { return 0; }
static inline HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef*) { return HAL_OK; }
static inline uint8_t HAL_SD_GetState_Ready() { return 1; }
#endif

namespace Peripherals {

// ======================= ServoDriver =======================

ServoDriver::ServoDriver(TIM_HandleTypeDef* htim, uint32_t channel)
    : htim_(htim), channel_(channel) {}

void ServoDriver::init(uint16_t pwm_min_us, uint16_t pwm_center_us, uint16_t pwm_max_us) {
    pwm_min_us_ = pwm_min_us;
    pwm_center_us_ = pwm_center_us;
    pwm_max_us_ = pwm_max_us;
    HAL_TIM_PWM_Start(htim_, channel_);
    setPulseWidthUs(pwm_center_us_); // safe default: centered
}

void ServoDriver::setPulseWidthUs(uint16_t us) {
    // Assumes timer prescaler/period configured so CCR counts == microseconds.
    // (e.g. 1 tick = 1us: timer clock / (prescaler+1) = 1MHz)
    __HAL_TIM_SET_COMPARE(htim_, channel_, us);
}

void ServoDriver::setDeflectionDeg(float deg, float min_deg, float max_deg) {
    float clamped = MathUtils::clamp(deg, min_deg, max_deg);

    // Linear map: min_deg -> pwm_min_us_, 0 -> pwm_center_us_, max_deg -> pwm_max_us_
    uint16_t pulse_us;
    if (clamped >= 0.0f) {
        float t = (max_deg > 1e-6f) ? (clamped / max_deg) : 0.0f;
        pulse_us = static_cast<uint16_t>(pwm_center_us_ + t * (pwm_max_us_ - pwm_center_us_));
    } else {
        float t = (min_deg < -1e-6f) ? (clamped / min_deg) : 0.0f; // note: both negative, t positive
        pulse_us = static_cast<uint16_t>(pwm_center_us_ - t * (pwm_center_us_ - pwm_min_us_));
    }
    setPulseWidthUs(pulse_us);
}

// ======================= StatusLED =======================

StatusLED::StatusLED(TIM_HandleTypeDef* htim, uint32_t channel)
    : htim_(htim), channel_(channel) {}

void StatusLED::init() {
    HAL_TIM_PWM_Start(htim_, channel_);
    writeRGB(0, 0, 0);
}

void StatusLED::writeRGB(uint8_t /*r*/, uint8_t /*g*/, uint8_t /*b*/) {
    // Real implementation encodes WS2812 GRB bit timing into a DMA
    // buffer of PWM compare values (each bit ~1.25us) and kicks off a
    // one-shot DMA transfer via HAL_TIM_PWM_Start_DMA. Left as an
    // integration point since it's DMA-buffer-format specific to your
    // timer clock configuration.
}

namespace {
    void colorToRGB(LedColor c, uint8_t& r, uint8_t& g, uint8_t& b) {
        switch (c) {
            case LedColor::GREEN:  r = 0;   g = 40;  b = 0;  break;
            case LedColor::YELLOW: r = 40;  g = 40;  b = 0;  break;
            case LedColor::RED:    r = 40;  g = 0;   b = 0;  break;
            case LedColor::BLUE:   r = 0;   g = 0;   b = 40; break;
            case LedColor::PURPLE: r = 30;  g = 0;   b = 30; break;
            case LedColor::OFF:
            default:               r = 0;   g = 0;   b = 0;  break;
        }
    }
}

void StatusLED::setColor(LedColor color) {
    solid_color_ = color;
    blink_active_ = false;
    uint8_t r, g, b;
    colorToRGB(color, r, g, b);
    writeRGB(r, g, b);
}

void StatusLED::setBlinkPattern(LedColor color, uint8_t blink_count) {
    blink_color_ = color;
    blink_count_ = blink_count;
    blink_active_ = true;
    blink_state_ = 0;
    last_transition_ms_ = 0;
}

void StatusLED::update(uint32_t now_ms) {
    if (!blink_active_) return;

    constexpr uint32_t BLINK_ON_MS = 150;
    constexpr uint32_t BLINK_OFF_MS = 150;
    constexpr uint32_t PATTERN_PAUSE_MS = 1000;

    uint32_t elapsed = now_ms - last_transition_ms_;
    uint8_t total_flashes = static_cast<uint8_t>(blink_count_ * 2); // on+off per flash

    if (blink_state_ < total_flashes) {
        bool is_on_phase = (blink_state_ % 2) == 0;
        uint32_t phase_duration = is_on_phase ? BLINK_ON_MS : BLINK_OFF_MS;
        if (elapsed >= phase_duration) {
            blink_state_++;
            last_transition_ms_ = now_ms;
            uint8_t r, g, b;
            if (blink_state_ < total_flashes) {
                bool now_on = (blink_state_ % 2) == 0;
                colorToRGB(now_on ? blink_color_ : LedColor::OFF, r, g, b);
            } else {
                colorToRGB(LedColor::OFF, r, g, b);
            }
            writeRGB(r, g, b);
        }
    } else {
        // pause between pattern repeats
        if (elapsed >= PATTERN_PAUSE_MS) {
            blink_state_ = 0;
            last_transition_ms_ = now_ms;
            uint8_t r, g, b;
            colorToRGB(blink_color_, r, g, b);
            writeRGB(r, g, b);
        }
    }
}

// ======================= SDCardIO =======================

SDCardIO::SDCardIO(SD_HandleTypeDef* hsd) : hsd_(hsd) {}

bool SDCardIO::init() {
    ready_ = (HAL_SD_Init(hsd_) == HAL_OK);
    return ready_;
}

// ======================= BatteryMonitor =======================

BatteryMonitor::BatteryMonitor(ADC_HandleTypeDef* hadc, uint32_t channel,
                                 float divider_ratio, float vref, uint16_t adc_max_counts)
    : hadc_(hadc), channel_(channel), divider_ratio_(divider_ratio),
      vref_(vref), adc_max_counts_(adc_max_counts) {}

float BatteryMonitor::readVoltage() {
    HAL_ADC_Start(hadc_);
    HAL_ADC_PollForConversion(hadc_, 10);
    uint32_t raw = HAL_ADC_GetValue(hadc_);
    float adc_voltage = (static_cast<float>(raw) / static_cast<float>(adc_max_counts_)) * vref_;
    return adc_voltage * divider_ratio_;
}

// ======================= ChargerMonitor =======================

ChargerMonitor::ChargerMonitor(I2C_HandleTypeDef* hi2c, uint8_t device_addr)
    : hi2c_(hi2c), device_addr_(device_addr) {}

bool ChargerMonitor::readRegister(uint8_t reg, uint8_t& out_value) {
    return HAL_I2C_Mem_Read(hi2c_, device_addr_ << 1, reg, 1, &out_value, 1, 10) == HAL_OK;
}

bool ChargerMonitor::init() {
    uint8_t dummy = 0;
    return readRegister(0x00, dummy); // verify device responds on the bus
}

ChargerState ChargerMonitor::readState() {
    uint8_t status_reg = 0;
    if (!readRegister(0x0B, status_reg)) { // BQ25883 status register (per datasheet)
        return ChargerState::UNKNOWN;
    }
    uint8_t chg_stat = (status_reg >> 3) & 0x03; // bit field per datasheet
    switch (chg_stat) {
        case 0: return ChargerState::NOT_CHARGING;
        case 1: return ChargerState::PRE_CHARGE;
        case 2: return ChargerState::FAST_CHARGE;
        case 3: return ChargerState::CHARGE_DONE;
        default: return ChargerState::UNKNOWN;
    }
}

} // namespace Peripherals
