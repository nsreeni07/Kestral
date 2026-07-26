// flight_logger.cpp
//
// FlightLogger: real implementation should sit on top of FatFs (f_open/
// f_write/f_sync) — placeholder writeToSD() below stands in for that.
// The buffering strategy (accumulate rows, flush at threshold) is the
// part that matters for SD wear/performance; swap in real FatFs calls
// in writeToSD()/begin() without touching the rest of this file.

#include "flight_logger.h"
#include "config.h"
#include <cstdio>
#include <cstring>

namespace Logging {

FlightLogger::FlightLogger(Peripherals::SDCardIO& sd, uint32_t buffer_size_bytes)
    : sd_(sd), buffer_capacity_(buffer_size_bytes) {
    if (buffer_capacity_ > kMaxBuffer) buffer_capacity_ = kMaxBuffer;
}

bool FlightLogger::writeToSD(const char* data, uint32_t length) {
    if (!sd_.isReady()) return false;
    // Placeholder: real implementation calls f_write(&file_, data, length, &written)
    // and f_sync(&file_) periodically (not every write, to limit wear).
    (void)data;
    (void)length;
    return true;
}

bool FlightLogger::begin() {
    if (!sd_.isReady()) return false;

    // Placeholder: real implementation increments a filename suffix by
    // probing f_stat() for FLIGHT001.CSV, FLIGHT002.CSV, ... and opens
    // the first available name with f_open(..., FA_CREATE_NEW | FA_WRITE).

    static const char* header =
        "timestamp_ms,altitude_m,pressure_hpa,temperature_c,"
        "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
        "roll_deg,pitch_deg,yaw_deg,"
        "pid_pitch,pid_yaw,servo_pitch_deg,servo_yaw_deg,"
        "flight_state,battery_voltage,active_faults\n";

    return writeToSD(header, static_cast<uint32_t>(std::strlen(header)));
}

void FlightLogger::appendRow(const Flight::FlightData& data) {
    // snprintf into the remaining buffer space. If the row would
    // overflow, flush first.
    if (buffer_used_ >= buffer_capacity_ - 256) {
        flush();
    }

    int written = std::snprintf(
        buffer_ + buffer_used_, kMaxBuffer - buffer_used_,
        "%lu,%.3f,%.3f,%.2f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%s,%.3f,%lu\n",
        static_cast<unsigned long>(data.timestamp_ms),
        data.altitude_m, data.pressure_hpa, data.temperature_c,
        data.accel_mps2.x, data.accel_mps2.y, data.accel_mps2.z,
        data.gyro_dps.x, data.gyro_dps.y, data.gyro_dps.z,
        data.roll_deg, data.pitch_deg, data.yaw_deg,
        data.pid_output_pitch, data.pid_output_yaw,
        data.servo_pitch_deg, data.servo_yaw_deg,
        Flight::stateToString(data.flight_state),
        data.battery_voltage,
        static_cast<unsigned long>(data.active_faults));

    if (written > 0) {
        buffer_used_ += static_cast<uint32_t>(written);
    }
}

bool FlightLogger::logRow(const Flight::FlightData& data) {
    appendRow(data);

    if (buffer_used_ >= buffer_capacity_) {
        return flush();
    }
    return true;
}

bool FlightLogger::flush() {
    if (buffer_used_ == 0) return true;
    bool ok = writeToSD(buffer_, buffer_used_);
    buffer_used_ = 0;
    return ok;
}

} // namespace Logging

namespace Utilities {

FaultManager::FaultManager(Peripherals::StatusLED& led) : led_(led) {}

uint8_t FaultManager::blinkCountForFault(Flight::FaultCode code) const {
    using Flight::FaultCode;
    if (Flight::hasFault(code, FaultCode::IMU_INIT_FAIL))    return 2;
    if (Flight::hasFault(code, FaultCode::BARO_INIT_FAIL))   return 3;
    if (Flight::hasFault(code, FaultCode::SD_INIT_FAIL))     return 4;
    if (Flight::hasFault(code, FaultCode::SD_WRITE_FAIL))    return 5;
    if (Flight::hasFault(code, FaultCode::IMU_COMM_TIMEOUT)) return 6;
    if (Flight::hasFault(code, FaultCode::BARO_COMM_TIMEOUT))return 7;
    if (Flight::hasFault(code, FaultCode::BROWNOUT_DETECTED))return 8;
    if (Flight::hasFault(code, FaultCode::WATCHDOG_TIMEOUT)) return 9;
    if (Flight::hasFault(code, FaultCode::BATTERY_CRITICAL)) return 10;
    return 1;
}

void FaultManager::raiseFault(Flight::FaultCode code, bool is_critical) {
    active_faults_ = active_faults_ | code;
    if (is_critical) {
        critical_fault_latched_ = true;
    }
    led_.setBlinkPattern(Peripherals::LedColor::RED, blinkCountForFault(code));
}

void FaultManager::clearFault(Flight::FaultCode code) {
    active_faults_ = static_cast<Flight::FaultCode>(
        static_cast<uint32_t>(active_faults_) & ~static_cast<uint32_t>(code));
}

void FaultManager::update(uint32_t now_ms) {
    led_.update(now_ms);
}

} // namespace Utilities
