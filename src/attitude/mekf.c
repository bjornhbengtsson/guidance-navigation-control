/*******************************************************************************
 *
 * FILE:
 *      mekf.c
 *
 * DESCRIPTION:
 *      Portable MEKF initialization implementation.
 *
 * This milestone initializes the nominal attitude, gyro-bias estimate, and
 * six-state local-error covariance. Prediction and measurement updates are
 * intentionally implemented in later milestones.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#include <math.h>
#include <stddef.h>

#include "gnc/mekf.h"

static bool mekf_standard_deviation_is_valid
    (
    float standard_deviation
    )
{
float variance;

if ( !isfinite(standard_deviation) ||
     standard_deviation < 0.0f )
    {
    return false;
    }

variance = standard_deviation * standard_deviation;

return isfinite(variance);

} /* mekf_standard_deviation_is_valid */

static bool mekf_standard_deviation_vector_is_valid
    (
    GNC_VECTOR3F standard_deviation
    )
{
return
    (
    mekf_standard_deviation_is_valid(standard_deviation.x) &&
    mekf_standard_deviation_is_valid(standard_deviation.y) &&
    mekf_standard_deviation_is_valid(standard_deviation.z)
    );

} /* mekf_standard_deviation_vector_is_valid */

static bool mekf_config_is_valid
    (
    const GNC_MEKF_CONFIG *config
    )
{
if ( config == NULL )
    {
    return false;
    }

if ( !mekf_standard_deviation_vector_is_valid
        (
        config->initial_attitude_std_rad
        ) )
    {
    return false;
    }

if ( !mekf_standard_deviation_vector_is_valid
        (
        config->initial_gyro_bias_std_rad_s
        ) )
    {
    return false;
    }

if ( !mekf_standard_deviation_is_valid
        (
        config->gyro_noise_density_rad_s_sqrt_hz
        ) )
    {
    return false;
    }

if ( !mekf_standard_deviation_is_valid
        (
        config->gyro_bias_random_walk_rad_s2_sqrt_hz
        ) )
    {
    return false;
    }

if ( !isfinite(config->maximum_delta_time_s) ||
     config->maximum_delta_time_s <= 0.0f )
    {
    return false;
    }

return true;

} /* mekf_config_is_valid */

static void mekf_clear_covariance
    (
    GNC_MEKF_FILTER *filter
    )
{
unsigned int row;
unsigned int column;

for ( row = 0U; row < GNC_MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U; column < GNC_MEKF_ERROR_STATE_DIM; column++ )
        {
        filter->covariance[row][column] = 0.0f;
        }
    }

} /* mekf_clear_covariance */

bool gnc_mekf_init
    (
    GNC_MEKF_FILTER *filter,
    GNC_QUATERNION initial_attitude_body_to_world,
    GNC_VECTOR3F initial_gyro_bias_body_rad_s,
    const GNC_MEKF_CONFIG *config
    )
{
GNC_MEKF_FILTER candidate;

if ( filter == NULL ||
     !gnc_quaternion_is_finite(initial_attitude_body_to_world) ||
     !gnc_vector3_is_finite(initial_gyro_bias_body_rad_s) ||
     !mekf_config_is_valid(config) )
    {
    return false;
    }

candidate.attitude_body_to_world =
    initial_attitude_body_to_world;

if ( !gnc_quaternion_normalize
        (
        &candidate.attitude_body_to_world
        ) )
    {
    return false;
    }

candidate.gyro_bias_body_rad_s =
    initial_gyro_bias_body_rad_s;

candidate.config = *config;

mekf_clear_covariance(&candidate);

candidate.covariance
    [GNC_MEKF_ATTITUDE_ERROR_X]
    [GNC_MEKF_ATTITUDE_ERROR_X] =
        config->initial_attitude_std_rad.x *
        config->initial_attitude_std_rad.x;

candidate.covariance
    [GNC_MEKF_ATTITUDE_ERROR_Y]
    [GNC_MEKF_ATTITUDE_ERROR_Y] =
        config->initial_attitude_std_rad.y *
        config->initial_attitude_std_rad.y;

candidate.covariance
    [GNC_MEKF_ATTITUDE_ERROR_Z]
    [GNC_MEKF_ATTITUDE_ERROR_Z] =
        config->initial_attitude_std_rad.z *
        config->initial_attitude_std_rad.z;

candidate.covariance
    [GNC_MEKF_GYRO_BIAS_ERROR_X]
    [GNC_MEKF_GYRO_BIAS_ERROR_X] =
        config->initial_gyro_bias_std_rad_s.x *
        config->initial_gyro_bias_std_rad_s.x;

candidate.covariance
    [GNC_MEKF_GYRO_BIAS_ERROR_Y]
    [GNC_MEKF_GYRO_BIAS_ERROR_Y] =
        config->initial_gyro_bias_std_rad_s.y *
        config->initial_gyro_bias_std_rad_s.y;

candidate.covariance
    [GNC_MEKF_GYRO_BIAS_ERROR_Z]
    [GNC_MEKF_GYRO_BIAS_ERROR_Z] =
        config->initial_gyro_bias_std_rad_s.z *
        config->initial_gyro_bias_std_rad_s.z;

*filter = candidate;

return true;

} /* gnc_mekf_init */
