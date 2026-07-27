/*******************************************************************************
 *
 * FILE:
 *      mahony.h
 *
 * DESCRIPTION:
 *      Portable Mahony attitude-filter interface.
 *
 * Quaternion ordering is [w, x, y, z].
 * Attitude represents the body-to-world rotation.
 * Gyroscope inputs are body-frame radians per second.
 * Accelerometer inputs are body-frame meters per second squared.
 *
 * Copyright (c) 2025 Sun Devil Rocketry.
 * Copyright (c) 2026 Bjorn Bengtsson.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#ifndef GNC_MAHONY_H
#define GNC_MAHONY_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "gnc/quaternion.h"
#include "gnc/vector3.h"

/**
 * @brief Mahony gains and measurement-validity limits.
 */
typedef struct _GNC_MAHONY_CONFIG
    {
    float proportional_gain;
    float integral_gain;

    /**
     * Minimum accepted accelerometer magnitude in meters per second squared.
     */
    float accel_min_magnitude_mps2;

    /**
     * Maximum accepted accelerometer magnitude in meters per second squared.
     */
    float accel_max_magnitude_mps2;

    /**
     * Per-axis integral correction limit in radians per second.
     */
    float integral_limit_rad_s;

    } GNC_MAHONY_CONFIG;

/**
 * @brief State for a portable Mahony attitude filter.
 */
typedef struct _GNC_MAHONY_FILTER
    {
    GNC_QUATERNION attitude_body_to_world;
    GNC_VECTOR3F integral_error_rad_s;
    GNC_MAHONY_CONFIG config;

    } GNC_MAHONY_FILTER;

/**
 * @brief Returns the initial configuration matching the SDR software baseline.
 */
GNC_MAHONY_CONFIG gnc_mahony_default_config
    (
    void
    );

/**
 * @brief Initializes the filter.
 *
 * The initial quaternion must be finite and have nonzero magnitude. A valid
 * non-unit quaternion is normalized.
 */
bool gnc_mahony_init
    (
    GNC_MAHONY_FILTER *filter,
    GNC_QUATERNION initial_attitude_body_to_world,
    const GNC_MAHONY_CONFIG *config
    );

/**
 * @brief Propagates attitude using body-frame gyro measurements.
 */
bool gnc_mahony_update_gyro
    (
    GNC_MAHONY_FILTER *filter,
    GNC_VECTOR3F gyro_body_rad_s,
    float delta_time_s
    );

/**
 * @brief Applies optional accelerometer correction and propagates attitude.
 *
 * Invalid or disabled accelerometer data is ignored and gyro propagation
 * continues. Integral correction updates only while accelerometer correction
 * is valid and enabled.
 */
bool gnc_mahony_update_imu
    (
    GNC_MAHONY_FILTER *filter,
    GNC_VECTOR3F gyro_body_rad_s,
    GNC_VECTOR3F accel_body_mps2,
    float delta_time_s,
    bool use_accel
    );

#ifdef __cplusplus
}
#endif

#endif /* GNC_MAHONY_H */
