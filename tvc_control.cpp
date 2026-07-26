// tvc_control.cpp

#include "tvc_control.h"
#include "config.h"

namespace Control {

// ======================= PIDController =======================

PIDController::PIDController(float kp, float ki, float kd,
                               float integral_limit, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_limit_(integral_limit), output_limit_(output_limit) {}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_error_ = false;
}

float PIDController::update(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float p_term = kp_ * error;

    // Integral, with clamping applied to the accumulated integral itself
    // (not just the final output) — this is the anti-windup mechanism:
    // once the integral saturates, further error in the same direction
    // stops accumulating, so recovery is immediate once error reverses.
    integral_ += error * dt;
    integral_ = MathUtils::clamp(integral_, -integral_limit_, integral_limit_);
    float i_term = ki_ * integral_;

    // Derivative (on error; acceptable here since setpoint is constant
    // at 0 for TVC — no derivative-kick concern from setpoint changes)
    float derivative = 0.0f;
    if (has_prev_error_ && dt > 1e-6f) {
        derivative = (error - prev_error_) / dt;
    }
    prev_error_ = error;
    has_prev_error_ = true;
    float d_term = kd_ * derivative;

    float output = p_term + i_term + d_term;
    return MathUtils::clamp(output, -output_limit_, output_limit_);
}

// ======================= ServoMixer =======================

ServoMixer::ServoMixer(float min_deg, float max_deg, float max_rate_deg_per_update)
    : min_deg_(min_deg), max_deg_(max_deg),
      max_rate_deg_per_update_(max_rate_deg_per_update) {}

void ServoMixer::reset() {
    last_output_deg_ = 0.0f;
}

float ServoMixer::apply(float commanded_deg) {
    float clamped_command = MathUtils::clamp(commanded_deg, min_deg_, max_deg_);

    float delta = clamped_command - last_output_deg_;
    delta = MathUtils::clamp(delta, -max_rate_deg_per_update_, max_rate_deg_per_update_);

    last_output_deg_ += delta;
    return last_output_deg_;
}

// ======================= TVCController =======================

TVCController::TVCController(Peripherals::ServoDriver& pitch_servo,
                                Peripherals::ServoDriver& yaw_servo)
    : pitch_pid_(Config::PID::PITCH_KP, Config::PID::PITCH_KI, Config::PID::PITCH_KD,
                 Config::PID::INTEGRAL_LIMIT_DEG, Config::PID::OUTPUT_LIMIT_DEG),
      yaw_pid_(Config::PID::YAW_KP, Config::PID::YAW_KI, Config::PID::YAW_KD,
               Config::PID::INTEGRAL_LIMIT_DEG, Config::PID::OUTPUT_LIMIT_DEG),
      pitch_mixer_(Config::Servo::MIN_DEFLECTION_DEG, Config::Servo::MAX_DEFLECTION_DEG,
                   Config::Servo::MAX_RATE_DEG_PER_UPDATE),
      yaw_mixer_(Config::Servo::MIN_DEFLECTION_DEG, Config::Servo::MAX_DEFLECTION_DEG,
                 Config::Servo::MAX_RATE_DEG_PER_UPDATE),
      pitch_servo_(pitch_servo), yaw_servo_(yaw_servo) {}

void TVCController::setGains(float pitch_kp, float pitch_ki, float pitch_kd,
                               float yaw_kp, float yaw_ki, float yaw_kd) {
    pitch_pid_.setGains(pitch_kp, pitch_ki, pitch_kd);
    yaw_pid_.setGains(yaw_kp, yaw_ki, yaw_kd);
}

void TVCController::update(float pitch_deg, float yaw_deg, float dt, bool tvc_authorized,
                             float& out_pid_pitch, float& out_pid_yaw,
                             float& out_servo_pitch_deg, float& out_servo_yaw_deg) {

    if (!tvc_authorized) {
        // Not in POWERED_ASCENT: hold servos centered, keep PID state
        // clean so there's no windup or derivative kick if TVC were to
        // become authorized again later in the same flight.
        if (was_authorized_last_tick_) {
            pitch_pid_.reset();
            yaw_pid_.reset();
            pitch_mixer_.reset();
            yaw_mixer_.reset();
        }
        was_authorized_last_tick_ = false;

        out_pid_pitch = 0.0f;
        out_pid_yaw = 0.0f;
        out_servo_pitch_deg = Config::Servo::CENTER_DEFLECTION_DEG;
        out_servo_yaw_deg = Config::Servo::CENTER_DEFLECTION_DEG;

        pitch_servo_.setDeflectionDeg(Config::Servo::CENTER_DEFLECTION_DEG,
                                       Config::Servo::MIN_DEFLECTION_DEG, Config::Servo::MAX_DEFLECTION_DEG);
        yaw_servo_.setDeflectionDeg(Config::Servo::CENTER_DEFLECTION_DEG,
                                     Config::Servo::MIN_DEFLECTION_DEG, Config::Servo::MAX_DEFLECTION_DEG);
        return;
    }

    was_authorized_last_tick_ = true;

    // Setpoint is 0 deg for both axes: we want the vehicle pointing
    // straight up relative to its current thrust axis reference.
    out_pid_pitch = pitch_pid_.update(0.0f, pitch_deg, dt);
    out_pid_yaw = yaw_pid_.update(0.0f, yaw_deg, dt);

    out_servo_pitch_deg = pitch_mixer_.apply(out_pid_pitch);
    out_servo_yaw_deg = yaw_mixer_.apply(out_pid_yaw);

    pitch_servo_.setDeflectionDeg(out_servo_pitch_deg,
                                   Config::Servo::MIN_DEFLECTION_DEG, Config::Servo::MAX_DEFLECTION_DEG);
    yaw_servo_.setDeflectionDeg(out_servo_yaw_deg,
                                 Config::Servo::MIN_DEFLECTION_DEG, Config::Servo::MAX_DEFLECTION_DEG);
}

} // namespace Control
