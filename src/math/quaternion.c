/*******************************************************************************
 *
 * FILE:
 *      quaternion.c
 *
 * DESCRIPTION:
 *      Portable quaternion implementation.
 *
 * Quaternion multiplication and frame-rotation behavior are derived from the
 * public Sun Devil Rocketry math_sdr module recorded in
 * upstream/sdr-rev2/SOURCE_REVISIONS.txt.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#include <math.h>
#include <stddef.h>

#include "gnc/quaternion.h"

GNC_QUATERNION gnc_quaternion_identity
    (
    void
    )
{
GNC_QUATERNION identity = { 1.0f, 0.0f, 0.0f, 0.0f };

return identity;

} /* gnc_quaternion_identity */

bool gnc_quaternion_is_finite
    (
    GNC_QUATERNION quaternion
    )
{
return
    (
    isfinite(quaternion.w) &&
    isfinite(quaternion.x) &&
    isfinite(quaternion.y) &&
    isfinite(quaternion.z)
    );

} /* gnc_quaternion_is_finite */

float gnc_quaternion_norm
    (
    GNC_QUATERNION quaternion
    )
{
return sqrtf
    (
    quaternion.w * quaternion.w +
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z
    );

} /* gnc_quaternion_norm */

bool gnc_quaternion_normalize
    (
    GNC_QUATERNION *quaternion
    )
{
float norm;

if ( quaternion == NULL ||
     !gnc_quaternion_is_finite(*quaternion) )
    {
    return false;
    }

norm = gnc_quaternion_norm(*quaternion);

if ( !isfinite(norm) ||
     norm <= 0.0f )
    {
    return false;
    }

quaternion->w /= norm;
quaternion->x /= norm;
quaternion->y /= norm;
quaternion->z /= norm;

return gnc_quaternion_is_finite(*quaternion);

} /* gnc_quaternion_normalize */

GNC_QUATERNION gnc_quaternion_multiply
    (
    GNC_QUATERNION a,
    GNC_QUATERNION b
    )
{
GNC_QUATERNION result;

result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

return result;

} /* gnc_quaternion_multiply */

float gnc_quaternion_dot
    (
    GNC_QUATERNION a,
    GNC_QUATERNION b
    )
{
return
    (
    a.w * b.w +
    a.x * b.x +
    a.y * b.y +
    a.z * b.z
    );

} /* gnc_quaternion_dot */

GNC_QUATERNION gnc_quaternion_add
    (
    GNC_QUATERNION a,
    GNC_QUATERNION b
    )
{
GNC_QUATERNION result;

result.w = a.w + b.w;
result.x = a.x + b.x;
result.y = a.y + b.y;
result.z = a.z + b.z;

return result;

} /* gnc_quaternion_add */

GNC_QUATERNION gnc_quaternion_scale
    (
    GNC_QUATERNION quaternion,
    float scalar
    )
{
GNC_QUATERNION result;

result.w = quaternion.w * scalar;
result.x = quaternion.x * scalar;
result.y = quaternion.y * scalar;
result.z = quaternion.z * scalar;

return result;

} /* gnc_quaternion_scale */

GNC_QUATERNION gnc_quaternion_conjugate
    (
    GNC_QUATERNION quaternion
    )
{
GNC_QUATERNION result;

result.w = quaternion.w;
result.x = -quaternion.x;
result.y = -quaternion.y;
result.z = -quaternion.z;

return result;

} /* gnc_quaternion_conjugate */

GNC_QUATERNION gnc_quaternion_from_euler_zyx
    (
    float yaw_rad,
    float pitch_rad,
    float roll_rad
    )
{
float cos_yaw;
float cos_pitch;
float cos_roll;
float sin_yaw;
float sin_pitch;
float sin_roll;
GNC_QUATERNION result;

cos_yaw = cosf(yaw_rad * 0.5f);
cos_pitch = cosf(pitch_rad * 0.5f);
cos_roll = cosf(roll_rad * 0.5f);

sin_yaw = sinf(yaw_rad * 0.5f);
sin_pitch = sinf(pitch_rad * 0.5f);
sin_roll = sinf(roll_rad * 0.5f);

result.w =
    cos_roll * cos_pitch * cos_yaw +
    sin_roll * sin_pitch * sin_yaw;

result.x =
    sin_roll * cos_pitch * cos_yaw -
    cos_roll * sin_pitch * sin_yaw;

result.y =
    cos_roll * sin_pitch * cos_yaw +
    sin_roll * cos_pitch * sin_yaw;

result.z =
    cos_roll * cos_pitch * sin_yaw -
    sin_roll * sin_pitch * cos_yaw;

return result;

} /* gnc_quaternion_from_euler_zyx */

bool gnc_rotate_body_to_world
    (
    GNC_QUATERNION attitude_body_to_world,
    GNC_VECTOR3F vector_body,
    GNC_VECTOR3F *vector_world
    )
{
GNC_QUATERNION vector_body_quaternion;
GNC_QUATERNION rotated;
GNC_QUATERNION attitude_conjugate;

if ( vector_world == NULL ||
     !gnc_quaternion_is_finite(attitude_body_to_world) ||
     !gnc_vector3_is_finite(vector_body) )
    {
    return false;
    }

if ( !gnc_quaternion_normalize(&attitude_body_to_world) )
    {
    return false;
    }

vector_body_quaternion.w = 0.0f;
vector_body_quaternion.x = vector_body.x;
vector_body_quaternion.y = vector_body.y;
vector_body_quaternion.z = vector_body.z;

attitude_conjugate =
    gnc_quaternion_conjugate(attitude_body_to_world);

rotated = gnc_quaternion_multiply
    (
    gnc_quaternion_multiply
        (
        attitude_body_to_world,
        vector_body_quaternion
        ),
    attitude_conjugate
    );

vector_world->x = rotated.x;
vector_world->y = rotated.y;
vector_world->z = rotated.z;

return gnc_vector3_is_finite(*vector_world);

} /* gnc_rotate_body_to_world */

bool gnc_rotate_world_to_body
    (
    GNC_QUATERNION attitude_body_to_world,
    GNC_VECTOR3F vector_world,
    GNC_VECTOR3F *vector_body
    )
{
GNC_QUATERNION vector_world_quaternion;
GNC_QUATERNION rotated;
GNC_QUATERNION attitude_conjugate;

if ( vector_body == NULL ||
     !gnc_quaternion_is_finite(attitude_body_to_world) ||
     !gnc_vector3_is_finite(vector_world) )
    {
    return false;
    }

if ( !gnc_quaternion_normalize(&attitude_body_to_world) )
    {
    return false;
    }

vector_world_quaternion.w = 0.0f;
vector_world_quaternion.x = vector_world.x;
vector_world_quaternion.y = vector_world.y;
vector_world_quaternion.z = vector_world.z;

attitude_conjugate =
    gnc_quaternion_conjugate(attitude_body_to_world);

rotated = gnc_quaternion_multiply
    (
    gnc_quaternion_multiply
        (
        attitude_conjugate,
        vector_world_quaternion
        ),
    attitude_body_to_world
    );

vector_body->x = rotated.x;
vector_body->y = rotated.y;
vector_body->z = rotated.z;

return gnc_vector3_is_finite(*vector_body);

} /* gnc_rotate_world_to_body */
