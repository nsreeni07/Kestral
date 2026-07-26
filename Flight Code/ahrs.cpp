// ahrs.cpp
// Madgwick IMU filter (gyro + accel only, no magnetometer — yaw will
// drift slowly over the flight but this is acceptable since TVC only
// needs accurate pitch/roll for stabilization; yaw reference doesn't
// affect thrust vectoring authority) and barometric altitude/velocity
// estimation.

#include "ahrs.h"
#include <cmath>
#include "sensors.h"

namespace AHRS {

MadgwickFilter::MadgwickFilter(float sample_dt)
    : beta_(0.1f), sample_dt_(sample_dt) {}

void MadgwickFilter::reset() {
    q_ = MathUtils::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

void MadgwickFilter::update(const MathUtils::Vector3& gyro_dps, const MathUtils::Vector3& accel_mps2) {
    // Convert gyro to rad/s
    float gx = MathUtils::degToRad(gyro_dps.x);
    float gy = MathUtils::degToRad(gyro_dps.y);
    float gz = MathUtils::degToRad(gyro_dps.z);

    float q0 = q_.w, q1 = q_.x, q2 = q_.y, q3 = q_.z;

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Only apply accelerometer feedback if the reading is a valid
    // (non-degenerate) vector — guards against a zero-magnitude glitch.
    float ax = accel_mps2.x, ay = accel_mps2.y, az = accel_mps2.z;
    float accel_norm = std::sqrt(ax * ax + ay * ay + az * az);

    if (accel_norm > 1e-6f) {
        float inv_norm = 1.0f / accel_norm;
        ax *= inv_norm; ay *= inv_norm; az *= inv_norm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        // Gradient descent algorithm corrective step
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
                    - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                    - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float s_norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (s_norm > 1e-8f) {
            float inv_s_norm = 1.0f / s_norm;
            s0 *= inv_s_norm; s1 *= inv_s_norm; s2 *= inv_s_norm; s3 *= inv_s_norm;

            qDot1 -= beta_ * s0;
            qDot2 -= beta_ * s1;
            qDot3 -= beta_ * s2;
            qDot4 -= beta_ * s3;
        }
    }

    // Integrate rate of change to yield new quaternion
    q0 += qDot1 * sample_dt_;
    q1 += qDot2 * sample_dt_;
    q2 += qDot3 * sample_dt_;
    q3 += qDot4 * sample_dt_;

    q_ = MathUtils::Quaternion(q0, q1, q2, q3);
    q_.normalize();
}

void MadgwickFilter::getEulerDeg(float& roll_deg, float& pitch_deg, float& yaw_deg) const {
    float roll_rad, pitch_rad, yaw_rad;
    q_.toEuler(roll_rad, pitch_rad, yaw_rad);
    roll_deg = MathUtils::radToDeg(roll_rad);
    pitch_deg = MathUtils::radToDeg(pitch_rad);
    yaw_deg = MathUtils::radToDeg(yaw_rad);
}

// ======================= AltitudeEstimator =======================

AltitudeEstimator::AltitudeEstimator(float sample_dt, float velocity_lpf_alpha)
    : sample_dt_(sample_dt), velocity_derivative_(velocity_lpf_alpha) {}

void AltitudeEstimator::reset() {
    altitude_m_ = 0.0f;
    vertical_velocity_mps_ = 0.0f;
    velocity_derivative_.reset();
}

void AltitudeEstimator::update(float pressure_hpa) {
    altitude_m_ = Sensors::BMP580::calculateAltitude(pressure_hpa, ground_pressure_hpa_);
    vertical_velocity_mps_ = velocity_derivative_.update(altitude_m_, sample_dt_);
}

} // namespace AHRS
