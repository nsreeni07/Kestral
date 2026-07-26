// flight_data.h
// The single shared state struct. Every module reads/writes through this
// rather than passing data peer-to-peer, which keeps coupling low and
// makes logging a simple snapshot operation.
//
// Concurrency note: on a single-core Cortex-M7 with a cooperative
// scheduler (see scheduler.h), tasks run to completion and don't preempt
// each other arbitrarily — but IMU/control ISRs can interrupt the main
// loop. Multi-field writes from an ISR context should wrap the write in
// __disable_irq()/__enable_irq() (or NVIC priority masking) to avoid a
// torn read from a lower-priority task. Single-field float writes on
// Cortex-M7 are atomic and don't need protection.

#pragma once

#include <cstdint>
#include "math_utils.h"

namespace Flight {

enum class State : uint8_t {
    IDLE = 0,
    ARMED,
    LAUNCH_DETECTED,
    POWERED_ASCENT,
    COAST,
    APOGEE,
    DESCENT,
    LANDED,
    FAULT // terminal safe state, entered on unrecoverable error
};

inline const char* stateToString(State s) {
    switch (s) {
        case State::IDLE:             return "IDLE";
        case State::ARMED:            return "ARMED";
        case State::LAUNCH_DETECTED:  return "LAUNCH_DETECTED";
        case State::POWERED_ASCENT:   return "POWERED_ASCENT";
        case State::COAST:            return "COAST";
        case State::APOGEE:           return "APOGEE";
        case State::DESCENT:          return "DESCENT";
        case State::LANDED:           return "LANDED";
        case State::FAULT:            return "FAULT";
        default:                      return "UNKNOWN";
    }
}

enum class FaultCode : uint32_t {
    NONE                = 0,
    IMU_INIT_FAIL       = 1 << 0,
    BARO_INIT_FAIL      = 1 << 1,
    SD_INIT_FAIL        = 1 << 2,
    SD_WRITE_FAIL       = 1 << 3,
    IMU_COMM_TIMEOUT    = 1 << 4,
    BARO_COMM_TIMEOUT   = 1 << 5,
    BROWNOUT_DETECTED   = 1 << 6,
    WATCHDOG_TIMEOUT    = 1 << 7,
    BATTERY_CRITICAL    = 1 << 8,
};

inline FaultCode operator|(FaultCode a, FaultCode b) {
    return static_cast<FaultCode>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline bool hasFault(FaultCode mask, FaultCode check) {
    return (static_cast<uint32_t>(mask) & static_cast<uint32_t>(check)) != 0;
}

// Snapshot of the entire flight computer state at a point in time.
// This is what gets logged every LOG_HZ and what every module consults.
struct FlightData {
    // Timing
    uint32_t timestamp_ms = 0;

    // Raw / derived sensor data
    MathUtils::Vector3 accel_mps2;      // body-frame acceleration
    MathUtils::Vector3 gyro_dps;        // body-frame angular rate (deg/s)
    float temperature_c = 0.0f;
    float pressure_hpa = 0.0f;
    float altitude_m = 0.0f;            // AGL, relative to pad reference
    float vertical_velocity_mps = 0.0f;

    // Orientation (from AHRS)
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    // Control outputs
    float pid_output_pitch = 0.0f;
    float pid_output_yaw = 0.0f;
    float servo_pitch_deg = 0.0f;
    float servo_yaw_deg = 0.0f;

    // Flight state
    State flight_state = State::IDLE;

    // Power
    float battery_voltage = 0.0f;

    // Faults
    FaultCode active_faults = FaultCode::NONE;

    // Reference pressure captured at ARM time (hPa). Defaults to standard
    // sea-level pressure until FlightStateMachine::arm() overwrites it
    // with an actual ground-level reading.
    float ground_pressure_hpa = 1013.25f;
};

} // namespace Flight
