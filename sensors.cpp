// sensors.cpp
// Implementation of ICM-45686 and BMP580 drivers.
//
// IMPORTANT: This targets STM32 HAL. Replace the placeholder HAL calls
// (marked below) with real HAL_SPI_/HAL_I2C_ calls once you swap in
// "stm32f7xx_hal.h" and your CubeMX-generated peripheral handles.
// Register addresses and scale factors below follow each datasheet;
// double check them against your specific part revision before flight.

#include "sensors.h"
#include <cstring>
#include <cmath>

// ---- Placeholder HAL types/functions -----------------------------------
// Remove this block and #include "stm32f7xx_hal.h" in a real build.
#ifndef HAL_PLACEHOLDERS_DEFINED
#define HAL_PLACEHOLDERS_DEFINED
struct SPI_HandleTypeDef {};
struct I2C_HandleTypeDef {};
enum HAL_StatusTypeDef { HAL_OK = 0, HAL_ERROR = 1, HAL_TIMEOUT = 2 };
static inline HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
static inline HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t, uint32_t) { return HAL_OK; }
static inline void HAL_GPIO_WritePin_CS(uint8_t, bool) {}
#endif
// -------------------------------------------------------------------------

namespace Sensors {

constexpr uint32_t SPI_TIMEOUT_MS = 5;
constexpr uint32_t I2C_TIMEOUT_MS = 10;

// ======================= ICM-45686 =======================

// Register map (subset — consult ICM-45686 datasheet for full map)
namespace ICM45686Reg {
    constexpr uint8_t WHO_AM_I      = 0x72;
    constexpr uint8_t WHO_AM_I_VAL  = 0xE9; // expected device ID
    constexpr uint8_t PWR_MGMT0     = 0x1F;
    constexpr uint8_t ACCEL_CONFIG0 = 0x21;
    constexpr uint8_t GYRO_CONFIG0  = 0x20;
    constexpr uint8_t ACCEL_DATA_X1 = 0x0B; // start of accel burst read
    constexpr uint8_t GYRO_DATA_X1  = 0x11; // start of gyro burst read
    constexpr uint8_t TEMP_DATA1    = 0x09;
}

ICM45686::ICM45686(SPI_HandleTypeDef* hspi, uint8_t cs_pin)
    : hspi_(hspi), cs_pin_(cs_pin) {}

void ICM45686::csLow()  { HAL_GPIO_WritePin_CS(cs_pin_, false); }
void ICM45686::csHigh() { HAL_GPIO_WritePin_CS(cs_pin_, true); }

bool ICM45686::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value }; // MSB=0 for write
    csLow();
    HAL_StatusTypeDef result = HAL_SPI_Transmit(hspi_, tx, 2, SPI_TIMEOUT_MS);
    csHigh();
    return result == HAL_OK;
}

bool ICM45686::readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length) {
    uint8_t tx = static_cast<uint8_t>(reg | 0x80); // MSB=1 for read
    csLow();
    HAL_StatusTypeDef r1 = HAL_SPI_Transmit(hspi_, &tx, 1, SPI_TIMEOUT_MS);
    HAL_StatusTypeDef r2 = HAL_SPI_Receive(hspi_, buffer, length, SPI_TIMEOUT_MS);
    csHigh();
    return (r1 == HAL_OK) && (r2 == HAL_OK);
}

bool ICM45686::init() {
    uint8_t who_am_i = 0;
    if (!readRegisters(ICM45686Reg::WHO_AM_I, &who_am_i, 1)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }
    if (who_am_i != ICM45686Reg::WHO_AM_I_VAL) {
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }

    // Enable accel + gyro in low-noise mode
    if (!writeRegister(ICM45686Reg::PWR_MGMT0, 0x0F)) {
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }

    // Configure full-scale range: +/-16g accel, +/-2000dps gyro
    // (bit fields per datasheet; values shown are representative)
    if (!writeRegister(ICM45686Reg::ACCEL_CONFIG0, 0x06)) {
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }
    if (!writeRegister(ICM45686Reg::GYRO_CONFIG0, 0x06)) {
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }

    // Scale factors for +/-16g / +/-2000dps at 16-bit resolution
    accel_scale_ = (16.0f * 9.80665f) / 32768.0f; // m/s^2 per LSB
    gyro_scale_  = 2000.0f / 32768.0f;            // deg/s per LSB

    status_ = SensorStatus::OK;
    return true;
}

bool ICM45686::readAcceleration(MathUtils::Vector3& out_mps2) {
    if (status_ != SensorStatus::OK) return false;

    uint8_t raw[6] = {0};
    if (!readRegisters(ICM45686Reg::ACCEL_DATA_X1, raw, 6)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }

    int16_t raw_x = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    int16_t raw_y = static_cast<int16_t>((raw[2] << 8) | raw[3]);
    int16_t raw_z = static_cast<int16_t>((raw[4] << 8) | raw[5]);

    out_mps2.x = static_cast<float>(raw_x) * accel_scale_;
    out_mps2.y = static_cast<float>(raw_y) * accel_scale_;
    out_mps2.z = static_cast<float>(raw_z) * accel_scale_;
    return true;
}

bool ICM45686::readGyroscope(MathUtils::Vector3& out_dps) {
    if (status_ != SensorStatus::OK) return false;

    uint8_t raw[6] = {0};
    if (!readRegisters(ICM45686Reg::GYRO_DATA_X1, raw, 6)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }

    int16_t raw_x = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    int16_t raw_y = static_cast<int16_t>((raw[2] << 8) | raw[3]);
    int16_t raw_z = static_cast<int16_t>((raw[4] << 8) | raw[5]);

    out_dps.x = static_cast<float>(raw_x) * gyro_scale_;
    out_dps.y = static_cast<float>(raw_y) * gyro_scale_;
    out_dps.z = static_cast<float>(raw_z) * gyro_scale_;
    return true;
}

bool ICM45686::readTemperature(float& out_celsius) {
    if (status_ != SensorStatus::OK) return false;

    uint8_t raw[2] = {0};
    if (!readRegisters(ICM45686Reg::TEMP_DATA1, raw, 2)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }
    int16_t raw_temp = static_cast<int16_t>((raw[0] << 8) | raw[1]);
    // Per datasheet: Temp(C) = (raw / 132.48) + 25
    out_celsius = (static_cast<float>(raw_temp) / 132.48f) + 25.0f;
    return true;
}

// ======================= BMP580 =======================

namespace BMP580Reg {
    constexpr uint8_t CHIP_ID       = 0x01;
    constexpr uint8_t CHIP_ID_VAL   = 0x50; // expected device ID
    constexpr uint8_t ODR_CONFIG    = 0x37;
    constexpr uint8_t OSR_CONFIG    = 0x36;
    constexpr uint8_t TEMP_DATA_XLSB = 0x1D; // start of temp burst read
    constexpr uint8_t PRESS_DATA_XLSB = 0x20; // start of pressure burst read
}

BMP580::BMP580(I2C_HandleTypeDef* hi2c, uint8_t device_addr)
    : hi2c_(hi2c), device_addr_(device_addr) {}

bool BMP580::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t v = value;
    return HAL_I2C_Mem_Write(hi2c_, device_addr_ << 1, reg, 1, &v, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool BMP580::readRegisters(uint8_t reg, uint8_t* buffer, uint16_t length) {
    return HAL_I2C_Mem_Read(hi2c_, device_addr_ << 1, reg, 1, buffer, length, I2C_TIMEOUT_MS) == HAL_OK;
}

bool BMP580::init() {
    uint8_t chip_id = 0;
    if (!readRegisters(BMP580Reg::CHIP_ID, &chip_id, 1)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }
    if (chip_id != BMP580Reg::CHIP_ID_VAL) {
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }

    // Configure oversampling (high precision) and output data rate.
    // Values below are representative — tune per datasheet section on
    // OSR/ODR trade-offs; higher OSR = better resolution, lower max ODR.
    if (!writeRegister(BMP580Reg::OSR_CONFIG, 0x2B)) {
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }
    if (!writeRegister(BMP580Reg::ODR_CONFIG, 0x02)) { // ~50Hz
        status_ = SensorStatus::INIT_FAILED;
        return false;
    }

    status_ = SensorStatus::OK;
    return true;
}

bool BMP580::readPressure(float& out_hpa) {
    if (status_ != SensorStatus::OK) return false;

    uint8_t raw[3] = {0};
    if (!readRegisters(BMP580Reg::PRESS_DATA_XLSB, raw, 3)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }
    // BMP580 outputs pressure pre-compensated in fixed-point Pa; the
    // sensor's internal DSP handles compensation (unlike older BMP280).
    uint32_t raw_press = (static_cast<uint32_t>(raw[2]) << 16) |
                          (static_cast<uint32_t>(raw[1]) << 8) |
                          raw[0];
    out_hpa = static_cast<float>(raw_press) / 64.0f / 100.0f; // Pa (Q6.16-ish) -> hPa
    return true;
}

bool BMP580::readTemperature(float& out_celsius) {
    if (status_ != SensorStatus::OK) return false;

    uint8_t raw[3] = {0};
    if (!readRegisters(BMP580Reg::TEMP_DATA_XLSB, raw, 3)) {
        status_ = SensorStatus::COMM_TIMEOUT;
        return false;
    }
    int32_t raw_temp = (static_cast<int32_t>(raw[2]) << 16) |
                        (static_cast<int32_t>(raw[1]) << 8) |
                        raw[0];
    out_celsius = static_cast<float>(raw_temp) / 65536.0f;
    return true;
}

float BMP580::calculateAltitude(float pressure_hpa, float ground_pressure_hpa) {
    // International barometric formula, referenced to ground pressure
    // captured at ARM time (gives altitude AGL rather than MSL).
    constexpr float EXPONENT = 0.1902949572f; // 1 / 5.25588
    return 44330.0f * (1.0f - std::pow(pressure_hpa / ground_pressure_hpa, EXPONENT));
}

} // namespace Sensors
