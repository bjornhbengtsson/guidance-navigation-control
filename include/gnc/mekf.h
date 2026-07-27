/*******************************************************************************
 *
 * FILE:
 *      mekf.h
 *
 * DESCRIPTION:
 *      Portable multiplicative extended Kalman filter interface.
 *
 * Quaternion ordering is [w, x, y, z].
 * Nominal attitude represents the body-to-world rotation.
 *
 * The six-state right-multiplicative local error vector is:
 *
 *     delta_x =
 *         [
 *         delta_theta_x,
 *         delta_theta_y,
 *         delta_theta_z,
 *         delta_bias_x,
 *         delta_bias_y,
 *         delta_bias_z
 *         ]
 *
 * Copyright (c) 2025 Sun Devil Rocketry.
 * Copyright (c) 2026 Bjorn Bengtsson.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#ifndef GNC_MEKF_H
#define GNC_MEKF_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "gnc/quaternion.h"
#include "gnc/vector3.h"

#define GNC_MEKF_ERROR_STATE_DIM    6U

typedef enum _GNC_MEKF_ERROR_STATE_INDEX
    {
    GNC_MEKF_ATTITUDE_ERROR_X = 0,
    GNC_MEKF_ATTITUDE_ERROR_Y,
    GNC_MEKF_ATTITUDE_ERROR_Z,
    GNC_MEKF_GYRO_BIAS_ERROR_X,
    GNC_MEKF_GYRO_BIAS_ERROR_Y,
    GNC_MEKF_GYRO_BIAS_ERROR_Z
    } GNC_MEKF_ERROR_STATE_INDEX;

/**
 * @brief Initial uncertainty and gyro-prediction configuration.
 */
typedef struct _GNC_MEKF_CONFIG
    {
    /**
     * Initial one-sigma local attitude uncertainty in radians.
     */
    GNC_VECTOR3F initial_attitude_std_rad;

    /**
     * Initial one-sigma gyro-bias uncertainty in radians per second.
     */
    GNC_VECTOR3F initial_gyro_bias_std_rad_s;

    /**
     * Continuous gyro white-noise density in rad/s/sqrt(Hz).
     */
    float gyro_noise_density_rad_s_sqrt_hz;

    /**
     * Continuous gyro-bias random-walk density in rad/s^2/sqrt(Hz).
     */
    float gyro_bias_random_walk_rad_s2_sqrt_hz;

    /**
     * Maximum accepted prediction timestep in seconds.
     */
    float maximum_delta_time_s;

    } GNC_MEKF_CONFIG;

/**
 * @brief Nominal state, local-error covariance, and configuration.
 *
 * The attitude error is a right-multiplicative local body-frame rotation:
 *
 *     attitude_true = attitude_nominal * delta_attitude
 *
 * Covariance is defined over the six-state local error vector, not over four
 * quaternion components.
 */
typedef struct _GNC_MEKF_FILTER
    {
    GNC_QUATERNION attitude_body_to_world;
    GNC_VECTOR3F gyro_bias_body_rad_s;

    float covariance
        [GNC_MEKF_ERROR_STATE_DIM]
        [GNC_MEKF_ERROR_STATE_DIM];

    GNC_MEKF_CONFIG config;

    } GNC_MEKF_FILTER;

/**
 * @brief Initializes a six-state attitude and gyro-bias MEKF.
 *
 * The initial attitude must be finite and have nonzero magnitude. A valid
 * non-unit quaternion is normalized. Covariance is initialized as a diagonal
 * matrix from the squared per-axis standard deviations.
 *
 * Initialization is transactional: on failure, the destination filter is not
 * modified.
 */
bool gnc_mekf_init
    (
    GNC_MEKF_FILTER *filter,
    GNC_QUATERNION initial_attitude_body_to_world,
    GNC_VECTOR3F initial_gyro_bias_body_rad_s,
    const GNC_MEKF_CONFIG *config
    );

#ifdef __cplusplus
}
#endif

#endif /* GNC_MEKF_H */
