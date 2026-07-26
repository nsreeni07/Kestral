// math_utils.h
// Header-only math primitives: Vector3, Quaternion, simple filters.
// Kept header-only (inline) since these are small, hot-path functions
// called at up to 1kHz — avoids call overhead and lets the compiler
// inline aggressively.

#pragma once

#include <cmath>
#include <cstdint>

namespace MathUtils {

// ---------------------------------------------------------------------
// Vector3
// ---------------------------------------------------------------------
struct Vector3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3() = default;
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vector3 operator*(float s) const { return {x * s, y * s, z * s}; }

    float magnitude() const { return std::sqrt(x * x + y * y + z * z); }

    Vector3 normalized() const {
        float mag = magnitude();
        if (mag < 1e-8f) return {0.0f, 0.0f, 0.0f};
        float inv = 1.0f / mag;
        return {x * inv, y * inv, z * inv};
    }
};

// ---------------------------------------------------------------------
// Quaternion (w, x, y, z) — Hamilton convention, used by Madgwick AHRS
// ---------------------------------------------------------------------
struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    void normalize() {
        float n = norm();
        if (n < 1e-8f) { w = 1.0f; x = y = z = 0.0f; return; }
        float inv = 1.0f / n;
        w *= inv; x *= inv; y *= inv; z *= inv;
    }

    // Convert to Euler angles (radians). Aerospace convention: roll (x),
    // pitch (y), yaw (z), ZYX intrinsic rotation order.
    void toEuler(float& roll, float& pitch, float& yaw) const {
        // roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (std::fabs(sinp) >= 1.0f) {
            pitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sinp); // gimbal lock clamp
        } else {
            pitch = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
};

constexpr float RAD_TO_DEG = 57.29577951308232f;
constexpr float DEG_TO_RAD = 0.017453292519943295f;

inline float radToDeg(float rad) { return rad * RAD_TO_DEG; }
inline float degToRad(float deg) { return deg * DEG_TO_RAD; }

inline float clamp(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

// ---------------------------------------------------------------------
// Single-pole low-pass filter — used for velocity derivative smoothing,
// servo output smoothing, etc.
// ---------------------------------------------------------------------
class LowPassFilter {
public:
    explicit LowPassFilter(float alpha = 0.2f) : alpha_(alpha) {}

    float update(float input) {
        if (!initialized_) {
            state_ = input;
            initialized_ = true;
            return state_;
        }
        state_ = alpha_ * input + (1.0f - alpha_) * state_;
        return state_;
    }

    float value() const { return state_; }
    void reset() { initialized_ = false; state_ = 0.0f; }

private:
    float alpha_;
    float state_ = 0.0f;
    bool initialized_ = false;
};

// ---------------------------------------------------------------------
// Simple numerical derivative with built-in low-pass on the result.
// Used to derive vertical velocity from barometric altitude.
// ---------------------------------------------------------------------
class DerivativeFilter {
public:
    explicit DerivativeFilter(float lpf_alpha = 0.2f) : lpf_(lpf_alpha) {}

    float update(float value, float dt) {
        float derivative = 0.0f;
        if (has_prev_ && dt > 1e-6f) {
            derivative = (value - prev_value_) / dt;
        }
        prev_value_ = value;
        has_prev_ = true;
        return lpf_.update(derivative);
    }

    void reset() { has_prev_ = false; lpf_.reset(); }

private:
    float prev_value_ = 0.0f;
    bool has_prev_ = false;
    LowPassFilter lpf_;
};

} // namespace MathUtils
