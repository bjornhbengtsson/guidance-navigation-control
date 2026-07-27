/*******************************************************************************
 *
 * FILE:
 *      quaternion.h
 *
 * DESCRIPTION:
 *      Portable quaternion interface for the GNC library.
 *
 * The quaternion ordering is [w, x, y, z].
 * Attitude quaternions represent the body-to-world rotation.
 *
 * Copyright (c) 2025 Sun Devil Rocketry.
 * Copyright (c) 2026 Bjorn Bengtsson.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#ifndef GNC_QUATERNION_H
#define GNC_QUATERNION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "gnc/vector3.h"

typedef struct _GNC_QUATERNION
    {
    float w;
    float x;
    float y;
    float z;
    } GNC_QUATERNION;

GNC_QUATERNION gnc_quaternion_identity
    (
    void
    );

bool gnc_quaternion_is_finite
    (
    GNC_QUATERNION quaternion
    );

float gnc_quaternion_norm
    (
    GNC_QUATERNION quaternion
    );

bool gnc_quaternion_normalize
    (
    GNC_QUATERNION *quaternion
    );

GNC_QUATERNION gnc_quaternion_multiply
    (
    GNC_QUATERNION a,
    GNC_QUATERNION b
    );

float gnc_quaternion_dot
    (
    GNC_QUATERNION a,
    GNC_QUATERNION b
    );

GNC_QUATERNION gnc_quaternion_add
    (
    GNC_QUATERNION a,
    GNC_QUATERNION b
    );

GNC_QUATERNION gnc_quaternion_scale
    (
    GNC_QUATERNION quaternion,
    float scalar
    );

GNC_QUATERNION gnc_quaternion_conjugate
    (
    GNC_QUATERNION quaternion
    );

GNC_QUATERNION gnc_quaternion_from_euler_zyx
    (
    float yaw_rad,
    float pitch_rad,
    float roll_rad
    );

bool gnc_rotate_body_to_world
    (
    GNC_QUATERNION attitude_body_to_world,
    GNC_VECTOR3F vector_body,
    GNC_VECTOR3F *vector_world
    );

bool gnc_rotate_world_to_body
    (
    GNC_QUATERNION attitude_body_to_world,
    GNC_VECTOR3F vector_world,
    GNC_VECTOR3F *vector_body
    );

#ifdef __cplusplus
}
#endif

#endif /* GNC_QUATERNION_H */
