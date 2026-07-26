// state_machine.h
// IDLE -> ARMED -> LAUNCH_DETECTED -> POWERED_ASCENT -> COAST -> APOGEE
//      -> DESCENT -> LANDED, with a FAULT terminal state reachable from
// anywhere. Transitions are driven by acceleration, altitude, and
// vertical velocity, each requiring a sustained condition (not a single
// sample) to reject noise and avoid false triggers.
//
// This class does NOT read sensors directly — it's fed already-computed
// values (accel magnitude, altitude, vertical velocity) each control
// tick and only owns transition logic + timing. Keeps it unit-testable
// with synthetic inputs.

#pragma once

#include <cstdint>
#include "flight_data.h"

namespace Flight {

class FlightStateMachine {
public:
    FlightStateMachine() = default;

    // Call once per control tick (500Hz) with the latest derived values.
    // now_ms should come from a monotonic millisecond tick (see scheduler.h).
    void update(uint32_t now_ms, float accel_magnitude_mps2,
                float altitude_m, float vertical_velocity_mps);

    // External commands
    void arm();          // IDLE -> ARMED (ground station / operator command)
    void forceFault();    // any state -> FAULT (called by FaultManager)
    void abort();         // any state -> FAULT, e.g. from a range-safety command

    State currentState() const { return state_; }

    // True only during POWERED_ASCENT — the sole window in which the
    // TVC controller is permitted to actuate the servos. This is the
    // single authoritative gate; tvc_control.cpp checks this every tick.
    bool tvcAuthorized() const { return state_ == State::POWERED_ASCENT; }

private:
    State state_ = State::IDLE;

    // Timers used to require sustained conditions before transitioning
    uint32_t condition_start_ms_ = 0;
    bool condition_active_ = false;

    // Tracks last altitude for the LANDED detector (needs a longer window)
    float landed_reference_altitude_m_ = 0.0f;
    uint32_t landed_window_start_ms_ = 0;
    bool landed_window_active_ = false;

    // Tracks apogee detection (sustained negative vertical velocity)
    uint32_t apogee_window_start_ms_ = 0;
    bool apogee_window_active_ = false;

    // Tracks burnout detection (sustained low acceleration after ascent)
    uint32_t burnout_window_start_ms_ = 0;
    bool burnout_window_active_ = false;

    void transitionTo(State new_state);
};

} // namespace Flight
