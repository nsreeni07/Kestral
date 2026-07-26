// state_machine.cpp
// Each transition requires a condition to hold for a configured duration
// (Config::Flight::*_DURATION_MS / *_CONFIRM_MS) before committing,
// which is the primary defense against false triggers from vibration,
// handling, or sensor noise.

#include "state_machine.h"
#include "config.h"

namespace Flight {

void FlightStateMachine::arm() {
    if (state_ == State::IDLE) {
        transitionTo(State::ARMED);
    }
}

void FlightStateMachine::forceFault() {
    transitionTo(State::FAULT);
}

void FlightStateMachine::abort() {
    transitionTo(State::FAULT);
}

void FlightStateMachine::transitionTo(State new_state) {
    state_ = new_state;
    // Reset all sustained-condition timers on any transition so the next
    // state starts its detection windows fresh.
    condition_active_ = false;
    apogee_window_active_ = false;
    burnout_window_active_ = false;
    landed_window_active_ = false;
}

void FlightStateMachine::update(uint32_t now_ms, float accel_magnitude_mps2,
                                  float altitude_m, float vertical_velocity_mps) {
    switch (state_) {

        case State::IDLE:
        case State::FAULT:
            // No automatic transitions out of IDLE (needs arm()) or
            // FAULT (terminal — requires ground reset/power cycle).
            break;

        case State::ARMED: {
            // Launch detection: sustained high acceleration
            if (accel_magnitude_mps2 >= Config::Flight::LAUNCH_ACCEL_THRESHOLD_MPS2) {
                if (!condition_active_) {
                    condition_active_ = true;
                    condition_start_ms_ = now_ms;
                } else if ((now_ms - condition_start_ms_) >= Config::Flight::LAUNCH_ACCEL_DURATION_MS) {
                    transitionTo(State::LAUNCH_DETECTED);
                }
            } else {
                condition_active_ = false;
            }
            break;
        }

        case State::LAUNCH_DETECTED: {
            // Single-tick confirmation state — immediately hands off to
            // POWERED_ASCENT. Kept as a distinct state (per spec) so a
            // discrete log event marks liftoff separately from the
            // continuous ascent phase.
            transitionTo(State::POWERED_ASCENT);
            break;
        }

        case State::POWERED_ASCENT: {
            // Burnout detection: sustained low acceleration (thrust ended)
            if (accel_magnitude_mps2 <= Config::Flight::BURNOUT_ACCEL_THRESHOLD_MPS2) {
                if (!burnout_window_active_) {
                    burnout_window_active_ = true;
                    burnout_window_start_ms_ = now_ms;
                } else if ((now_ms - burnout_window_start_ms_) >= Config::Flight::BURNOUT_CONFIRM_MS) {
                    transitionTo(State::COAST);
                }
            } else {
                burnout_window_active_ = false;
            }
            break;
        }

        case State::COAST: {
            // Apogee detection: sustained non-positive vertical velocity
            if (vertical_velocity_mps <= Config::Flight::APOGEE_VELOCITY_THRESHOLD_MPS) {
                if (!apogee_window_active_) {
                    apogee_window_active_ = true;
                    apogee_window_start_ms_ = now_ms;
                } else if ((now_ms - apogee_window_start_ms_) >= Config::Flight::APOGEE_CONFIRM_MS) {
                    transitionTo(State::APOGEE);
                }
            } else {
                apogee_window_active_ = false;
            }
            break;
        }

        case State::APOGEE: {
            // Single-tick confirmation state -> DESCENT. Distinct state
            // gives dual-deployment logic (future expansion) a clean
            // hook point for drogue deployment.
            transitionTo(State::DESCENT);
            break;
        }

        case State::DESCENT: {
            // Landed detection: altitude stops changing for a sustained
            // window.
            if (!landed_window_active_) {
                landed_window_active_ = true;
                landed_window_start_ms_ = now_ms;
                landed_reference_altitude_m_ = altitude_m;
            } else {
                float delta = altitude_m - landed_reference_altitude_m_;
                if (delta < 0.0f) delta = -delta;

                if (delta > Config::Flight::LANDED_ALT_DELTA_M) {
                    // Still moving significantly — reset the window
                    landed_window_start_ms_ = now_ms;
                    landed_reference_altitude_m_ = altitude_m;
                } else if ((now_ms - landed_window_start_ms_) >= Config::Flight::LANDED_CONFIRM_MS) {
                    transitionTo(State::LANDED);
                }
            }
            break;
        }

        case State::LANDED:
            // Terminal nominal state — no further transitions expected.
            break;
    }
}

} // namespace Flight
