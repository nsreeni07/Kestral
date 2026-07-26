// tvc_control.h
// Generic PID controller + the TVC-specific wrapper that runs separate
// pitch and yaw PID loops, applies servo rate limiting/smoothing, and
// drives the two ServoDriver outputs. TVC is only ever active when
// FlightStateMachine::tvcAuthorized() is true — that gate is checked
// every tick in TVCController::update(), not just once at startup.

#pragma once

#include "math_utils.h"
#include "peripherals.h"

namespace Control {

// ---------------------------------------------------------------------
// PIDController — generic, reusable single-axis PID with anti-windup
// and output clamping.
// ---------------------------------------------------------------------
class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                   float integral_limit, float output_limit);

    // setpoint is typically 0 (target: zero pitch/yaw error from
    // vertical), measurement is the current angle in degrees.
    float update(float setpoint, float measurement, float dt);

    void reset();

    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

private:
    float kp_, ki_, kd_;
    float integral_limit_;
    float output_limit_;

    float integral_ = 0.0f;
    float prev_error_ = 0.0f;
    bool has_prev_error_ = false;
};

// ---------------------------------------------------------------------
// ServoMixer — takes a raw PID output (degrees) and applies rate
// limiting + smoothing before it reaches the actual servo, so a control
// transient can't slam the servo past its mechanical slew capability.
// ---------------------------------------------------------------------
class ServoMixer {
public:
    ServoMixer(float min_deg, float max_deg, float max_rate_deg_per_update);

    // Returns the rate-limited, clamped output to actually command.
    float apply(float commanded_deg);

    void reset();

private:
    float min_deg_, max_deg_;
    float max_rate_deg_per_update_;
    float last_output_deg_ = 0.0f;
};

// ---------------------------------------------------------------------
// TVCController — owns pitch + yaw PID loops, servo mixers, and the two
// ServoDriver instances. This is the only class permitted to command
// the TVC servos.
// ---------------------------------------------------------------------
class TVCController {
public:
    TVCController(Peripherals::ServoDriver& pitch_servo,
                  Peripherals::ServoDriver& yaw_servo);

    // Call once per control tick (Config::Rates::CONTROL_HZ).
    // roll/pitch/yaw_deg come from AHRS::MadgwickFilter.
    // tvc_authorized comes from FlightStateMachine::tvcAuthorized() —
    // if false, servos are commanded to center and PID state is held
    // reset so there's no integral windup or output kick when TVC
    // re-authorizes (which, by design, it never does after leaving
    // POWERED_ASCENT, but this keeps the logic safe regardless).
    void update(float pitch_deg, float yaw_deg, float dt, bool tvc_authorized,
                float& out_pid_pitch, float& out_pid_yaw,
                float& out_servo_pitch_deg, float& out_servo_yaw_deg);

    void setGains(float pitch_kp, float pitch_ki, float pitch_kd,
                  float yaw_kp, float yaw_ki, float yaw_kd);

private:
    PIDController pitch_pid_;
    PIDController yaw_pid_;
    ServoMixer pitch_mixer_;
    ServoMixer yaw_mixer_;
    Peripherals::ServoDriver& pitch_servo_;
    Peripherals::ServoDriver& yaw_servo_;

    bool was_authorized_last_tick_ = false;
};

} // namespace Control
