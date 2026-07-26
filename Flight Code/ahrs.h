// ahrs.h
// Sensor fusion: Madgwick orientation filter + barometric altitude /
// vertical velocity estimation. Consumes raw sensor readings (from
// sensors.h) and produces the derived quantities the state machine and
// TVC controller actually need.

#pragma once

#include "math_utils.h"

namespace AHRS {

// ---------------------------------------------------------------------
// MadgwickFilter
//
// Why Madgwick: it's a gradient-descent correction on gyro-integrated
// orientation using the accelerometer as a gravity reference, without
// requiring a matrix inversion (unlike a full EKF) — cheap enough to
// run at 1kHz on an F7 with headroom for everything else. A single beta
// gain controls the accel/gyro trust trade-off, which we schedule based
// on flight state: high beta on the pad (fast convergence while the
// accelerometer is a trustworthy gravity vector), low beta during
// powered ascent (thrust contaminates the accel reading, so we lean on
// the gyro instead), and higher again under canopy during descent.
// ---------------------------------------------------------------------
class MadgwickFilter {
public:
    explicit MadgwickFilter(float sample_dt);

    // Sets the beta gain — call this whenever flight state changes
    // (see Config::AHRS::BETA_* and the schedule in flight logic).
    void setBeta(float beta) { beta_ = beta; }

    // Runs one filter update. Gyro in deg/s, accel in m/s^2 (only
    // direction is used, magnitude is normalized internally).
    void update(const MathUtils::Vector3& gyro_dps, const MathUtils::Vector3& accel_mps2);

    // Extracts Euler angles from the current orientation estimate.
    void getEulerDeg(float& roll_deg, float& pitch_deg, float& yaw_deg) const;

    const MathUtils::Quaternion& orientation() const { return q_; }

    void reset();

private:
    MathUtils::Quaternion q_;
    float beta_;
    float sample_dt_;
};

// ---------------------------------------------------------------------
// AltitudeEstimator
//
// Converts barometric pressure to AGL altitude (referenced to a ground
// pressure captured at ARM time) and derives vertical velocity via a
// filtered numerical derivative. Vertical velocity is what the state
// machine actually uses for apogee detection (altitude alone is too
// noisy and lags the true apex).
// ---------------------------------------------------------------------
class AltitudeEstimator {
public:
    explicit AltitudeEstimator(float sample_dt, float velocity_lpf_alpha);

    // Call once at ARM time to set the zero-altitude reference.
    void setGroundPressure(float ground_pressure_hpa) { ground_pressure_hpa_ = ground_pressure_hpa; }

    // Feed a new pressure reading (hPa); updates internal altitude and
    // vertical velocity estimates.
    void update(float pressure_hpa);

    float altitudeM() const { return altitude_m_; }
    float verticalVelocityMps() const { return vertical_velocity_mps_; }

    void reset();

private:
    float sample_dt_;
    float ground_pressure_hpa_ = 1013.25f;
    float altitude_m_ = 0.0f;
    float vertical_velocity_mps_ = 0.0f;
    MathUtils::DerivativeFilter velocity_derivative_;
};

} // namespace AHRS
