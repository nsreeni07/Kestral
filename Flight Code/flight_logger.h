// flight_logger.h
// Buffered CSV flight data logger + central fault manager. Grouped
// together since both ultimately write status somewhere (SD card / LED)
// and both are called from the same places (state machine, sensor
// drivers) when something goes wrong.

#pragma once

#include <cstdint>
#include "flight_data.h"
#include "peripherals.h"

namespace Logging {

// ---------------------------------------------------------------------
// FlightLogger — buffers CSV rows in RAM and flushes to SD in blocks to
// minimize write wear and avoid blocking the control loop on slow SD
// writes every tick.
// ---------------------------------------------------------------------
class FlightLogger {
public:
    explicit FlightLogger(Peripherals::SDCardIO& sd, uint32_t buffer_size_bytes);

    // Opens a new log file (auto-incrementing filename) and writes the
    // CSV header row. Returns false on failure (caller should raise
    // FaultCode::SD_INIT_FAIL).
    bool begin();

    // Appends one row built from the current FlightData snapshot into
    // the RAM buffer. Call at Config::Rates::LOG_HZ.
    // Returns false only if a flush was attempted and failed.
    bool logRow(const Flight::FlightData& data);

    // Forces a flush of any buffered data to SD (call on state
    // transitions to LANDED / FAULT so nothing is lost).
    bool flush();

private:
    Peripherals::SDCardIO& sd_;
    uint32_t buffer_capacity_;

    static constexpr uint32_t kMaxBuffer = 8192;
    char buffer_[kMaxBuffer];
    uint32_t buffer_used_ = 0;

    bool writeToSD(const char* data, uint32_t length);
    void appendRow(const Flight::FlightData& data);
};

} // namespace Logging

namespace Utilities {

// ---------------------------------------------------------------------
// FaultManager — single place that owns "what do we do when something
// breaks" policy: log it, show it on the LED, and (for critical faults)
// disable TVC via the state machine.
// ---------------------------------------------------------------------
class FaultManager {
public:
    explicit FaultManager(Peripherals::StatusLED& led);

    // Raises a fault. Non-critical faults (e.g. a single dropped SD
    // write) are logged/indicated but don't force FAULT state.
    // Critical faults (sensor init failure, watchdog timeout, brownout)
    // call the provided callback to force the state machine into FAULT.
    void raiseFault(Flight::FaultCode code, bool is_critical);

    Flight::FaultCode activeFaults() const { return active_faults_; }
    bool hasCriticalFault() const { return critical_fault_latched_; }

    void clearFault(Flight::FaultCode code);

    // Call periodically (Config::Rates::STATUS_HZ) to drive LED blink
    // pattern reflecting current fault state.
    void update(uint32_t now_ms);

private:
    Peripherals::StatusLED& led_;
    Flight::FaultCode active_faults_ = Flight::FaultCode::NONE;
    bool critical_fault_latched_ = false;

    // Maps a fault code to a blink count (number of flashes) so the
    // LED can encode which fault occurred without a display or serial
    // link — useful for post-flight/pad diagnostics.
    uint8_t blinkCountForFault(Flight::FaultCode code) const;
};

} // namespace Utilities
