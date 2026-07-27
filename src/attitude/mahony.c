/*******************************************************************************
 *
 * FILE:
 *      mahony.c
 *
 * DESCRIPTION:
 *      Portable Mahony attitude-filter implementation.
 *
 * The algorithmic behavior is derived from the public Sun Devil Rocketry
 * Mahony implementation recorded in upstream/sdr-rev2/SOURCE_REVISIONS.txt.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#include <math.h>
#include <stddef.h>

#include "gnc/mahony.h"

#define GNC_STANDARD_GRAVITY_MPS2    9.8f

static float clamp_float
    (
    float value,
    float minimum,
    float maximum
    )
{
if ( value < minimum )
    {
    return minimum;
    }

if ( value > maximum )
    {
    return maximum;
    }

return value;

} /* clamp_float */

static bool mahony_config_is_valid
    (
    const GNC_MAHONY_CONFIG *config
    )
{
if ( config == NULL )
    {
    return false;
    }

if ( !isfinite(config->proportional_gain) ||
     !isfinite(config->integral_gain) ||
     !isfinite(config->accel_min_magnitude_mps2) ||
     !isfinite(config->accel_max_magnitude_mps2) ||
     !isfinite(config->integral_limit_rad_s) )
    {
    return false;
    }

if ( config->proportional_gain < 0.0f ||
     config->integral_gain < 0.0f ||
     config->accel_min_magnitude_mps2 <= 0.0f ||
     config->accel_max_magnitude_mps2 <
        config->accel_min_magnitude_mps2 ||
     config->integral_limit_rad_s < 0.0f )
    {
    return false;
    }

return true;

} /* mahony_config_is_valid */

static bool mahony_filter_is_valid
    (
    const GNC_MAHONY_FILTER *filter
    )
{
if ( filter == NULL ||
     !gnc_quaternion_is_finite(filter->attitude_body_to_world) ||
     !gnc_vector3_is_finite(filter->integral_error_rad_s) ||
     !mahony_config_is_valid(&filter->config) )
    {
    return false;
    }

return gnc_quaternion_norm(filter->attitude_body_to_world) > 0.0f;

} /* mahony_filter_is_valid */

GNC_MAHONY_CONFIG gnc_mahony_default_config
    (
    void
    )
{
GNC_MAHONY_CONFIG config;

config.proportional_gain = 1.0f;
config.integral_gain = 0.0f;
config.accel_min_magnitude_mps2 = 0.85f * GNC_STANDARD_GRAVITY_MPS2;
config.accel_max_magnitude_mps2 = 1.15f * GNC_STANDARD_GRAVITY_MPS2;
config.integral_limit_rad_s = 0.25f;

return config;

} /* gnc_mahony_default_config */

bool gnc_mahony_init
    (
    GNC_MAHONY_FILTER *filter,
    GNC_QUATERNION initial_attitude_body_to_world,
    const GNC_MAHONY_CONFIG *config
    )
{
GNC_QUATERNION normalized_attitude;

if ( filter == NULL ||
     !mahony_config_is_valid(config) ||
     !gnc_quaternion_is_finite(initial_attitude_body_to_world) )
    {
    return false;
    }

normalized_attitude = initial_attitude_body_to_world;

if ( !gnc_quaternion_normalize(&normalized_attitude) )
    {
    return false;
    }

filter->attitude_body_to_world = normalized_attitude;
filter->integral_error_rad_s.x = 0.0f;
filter->integral_error_rad_s.y = 0.0f;
filter->integral_error_rad_s.z = 0.0f;
filter->config = *config;

return true;

} /* gnc_mahony_init */

bool gnc_mahony_update_gyro
    (
    GNC_MAHONY_FILTER *filter,
    GNC_VECTOR3F gyro_body_rad_s,
    float delta_time_s
    )
{
GNC_QUATERNION angular_velocity;
GNC_QUATERNION attitude_derivative;
GNC_QUATERNION attitude_candidate;

if ( !mahony_filter_is_valid(filter) ||
     !gnc_vector3_is_finite(gyro_body_rad_s) ||
     !isfinite(delta_time_s) ||
     delta_time_s <= 0.0f )
    {
    return false;
    }

angular_velocity.w = 0.0f;
angular_velocity.x = gyro_body_rad_s.x;
angular_velocity.y = gyro_body_rad_s.y;
angular_velocity.z = gyro_body_rad_s.z;

/*
 * The attitude is body-to-world and angular velocity is expressed in body:
 *
 *     q_dot = 0.5 * q * omega_body
 */
attitude_derivative = gnc_quaternion_multiply
    (
    filter->attitude_body_to_world,
    angular_velocity
    );

attitude_derivative = gnc_quaternion_scale
    (
    attitude_derivative,
    0.5f * delta_time_s
    );

attitude_candidate = gnc_quaternion_add
    (
    filter->attitude_body_to_world,
    attitude_derivative
    );

if ( !gnc_quaternion_normalize(&attitude_candidate) )
    {
    return false;
    }

filter->attitude_body_to_world = attitude_candidate;

return true;

} /* gnc_mahony_update_gyro */

bool gnc_mahony_update_imu
    (
    GNC_MAHONY_FILTER *filter,
    GNC_VECTOR3F gyro_body_rad_s,
    GNC_VECTOR3F accel_body_mps2,
    float delta_time_s,
    bool use_accel
    )
{
GNC_MAHONY_FILTER candidate;
GNC_VECTOR3F gravity_estimated_body;
GNC_VECTOR3F attitude_error;
GNC_VECTOR3F proportional_correction;
GNC_VECTOR3F gyro_corrected;
float accel_magnitude;
bool accel_valid;

if ( !mahony_filter_is_valid(filter) ||
     !gnc_vector3_is_finite(gyro_body_rad_s) ||
     !isfinite(delta_time_s) ||
     delta_time_s <= 0.0f )
    {
    return false;
    }

candidate = *filter;
gyro_corrected = gyro_body_rad_s;

accel_magnitude = gnc_vector3_magnitude(accel_body_mps2);

accel_valid =
    gnc_vector3_is_finite(accel_body_mps2) &&
    isfinite(accel_magnitude) &&
    accel_magnitude >= candidate.config.accel_min_magnitude_mps2 &&
    accel_magnitude <= candidate.config.accel_max_magnitude_mps2;

if ( use_accel &&
     accel_valid &&
     gnc_vector3_normalize(&accel_body_mps2) )
    {
    if ( !gnc_rotate_world_to_body
            (
            candidate.attitude_body_to_world,
            (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
            &gravity_estimated_body
            ) )
        {
        return false;
        }

    /*
     * measured x estimated drives the predicted gravity direction toward the
     * measured direction for the current body-to-world convention.
     */
    attitude_error = gnc_vector3_cross
        (
        accel_body_mps2,
        gravity_estimated_body
        );

    if ( candidate.config.integral_gain > 0.0f )
        {
        candidate.integral_error_rad_s.x +=
            candidate.config.integral_gain *
            attitude_error.x *
            delta_time_s;

        candidate.integral_error_rad_s.y +=
            candidate.config.integral_gain *
            attitude_error.y *
            delta_time_s;

        candidate.integral_error_rad_s.z +=
            candidate.config.integral_gain *
            attitude_error.z *
            delta_time_s;

        candidate.integral_error_rad_s.x = clamp_float
            (
            candidate.integral_error_rad_s.x,
            -candidate.config.integral_limit_rad_s,
            candidate.config.integral_limit_rad_s
            );

        candidate.integral_error_rad_s.y = clamp_float
            (
            candidate.integral_error_rad_s.y,
            -candidate.config.integral_limit_rad_s,
            candidate.config.integral_limit_rad_s
            );

        candidate.integral_error_rad_s.z = clamp_float
            (
            candidate.integral_error_rad_s.z,
            -candidate.config.integral_limit_rad_s,
            candidate.config.integral_limit_rad_s
            );
        }

    proportional_correction = gnc_vector3_scale
        (
        attitude_error,
        candidate.config.proportional_gain
        );

    gyro_corrected = gnc_vector3_add
        (
        gyro_corrected,
        proportional_correction
        );

    gyro_corrected = gnc_vector3_add
        (
        gyro_corrected,
        candidate.integral_error_rad_s
        );
    }

if ( !gnc_mahony_update_gyro
        (
        &candidate,
        gyro_corrected,
        delta_time_s
        ) )
    {
    return false;
    }

*filter = candidate;

return true;

} /* gnc_mahony_update_imu */
