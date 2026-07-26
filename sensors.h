// sensors.h
// Raw sensor drivers: ICM-45686 (6-axis IMU) and BMP580 (barometer).
// These classes only know how to talk to the hardware and return
// scaled physical units — no fusion, no altitude math. That logic
// lives in ahrs.h, which consumes these.

#pragma once

#include <cstdint>
#include "math_utils.h"

// Forward-declare HAL handle types so this header doesn't force every
// including file to pull in the full HAL stack. Replace with the actual
// includes ("stm32f7xx_hal.h") in sensors.cpp once wired to CubeMX.
struct SPI_HandleTypeDef;
struct I2C_HandleTypeDef;

namespace Sensors {

enum class SensorStatus : uint8_t {
    OK = 0,
    INIT_FAILED,
    COMM_TIMEOUT,
    NOT_INITIALIZED
};

// ---------------------------------------------------------------------
// ICM-45686 — 6-axis IMU (SPI)
// ---------------------------------------------------------------------
class ICM45686 {
public:
    explicit ICM45686(SPI_HandleTypeDef* hspi, uint8_t cs_pin);

    // Verifies WHO_AM_I, configures ODR/full-scale range, enables sensors.
    // Returns false if the device does not respond or ID mismatch.
    bool init();

    // Reads raw accel registers, converts to m/s^2 using configured
    // full-scale range. Returns false on SPI timeout/error.
    bool readAcceleration(MathUtils::Vector3& out_mps2);

    // Reads raw gyro registers, converts to deg/s.
    bool readGyroscope(MathUtils::Vector3& out_dps);

    // Reads onboard die temperature (deg C) — useful for fault detection
    // (e.g. flagging thermal runaway near pyro channels), not for
    // ambient temperature (that's BMP580's job).
    bool readTemperature(float& out_celsius);

    SensorStatus status() const { return status_; }

private:
    SPI_HandleTypeDef* hspi_;
    uint8_t cs_pin_;
    SensorStatus status_ = SensorStatus::NOT_INITIALIZED;

    // Full-scale range scale factors, set during init() based on
    // configured range (e.g. +/-16g, +/-2000dps)
    float accel_scale_ = 0.0f; // (m/s^2) per LSB
    float gyro_scale_ = 0.0f;  // (deg/s) per LSB

    void csLow();
    void csHigh();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length);
};

// ---------------------------------------------------------------------
// BMP580 — barometric pressure + temperature sensor (I2C)
// ---------------------------------------------------------------------
class BMP580 {
public:
    explicit BMP580(I2C_HandleTypeDef* hi2c, uint8_t device_addr);

    // Verifies chip ID, loads factory calibration coefficients,
    // configures oversampling/ODR.
    bool init();

    // Reads compensated pressure in hPa.
    bool readPressure(float& out_hpa);

    // Reads compensated temperature in deg C.
    bool readTemperature(float& out_celsius);

    // Converts a pressure reading to altitude AGL using the international
    // barometric formula, referenced against ground_pressure_hpa
    // (captured at ARM time by the state machine).
    static float calculateAltitude(float pressure_hpa, float ground_pressure_hpa);

    SensorStatus status() const { return status_; }

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t device_addr_;
    SensorStatus status_ = SensorStatus::NOT_INITIALIZED;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length);
};

} // namespace Sensors
