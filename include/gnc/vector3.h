/*******************************************************************************
 *
 * FILE:
 *      vector3.h
 *
 * DESCRIPTION:
 *      Portable three-dimensional vector interface for the GNC library.
 *
 * Copyright (c) 2025 Sun Devil Rocketry.
 * Copyright (c) 2026 Bjorn Bengtsson.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#ifndef GNC_VECTOR3_H
#define GNC_VECTOR3_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

typedef struct _GNC_VECTOR3F
    {
    float x;
    float y;
    float z;
    } GNC_VECTOR3F;

bool gnc_vector3_is_finite
    (
    GNC_VECTOR3F vector
    );

float gnc_vector3_magnitude
    (
    GNC_VECTOR3F vector
    );

bool gnc_vector3_normalize
    (
    GNC_VECTOR3F *vector
    );

GNC_VECTOR3F gnc_vector3_add
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    );

GNC_VECTOR3F gnc_vector3_subtract
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    );

GNC_VECTOR3F gnc_vector3_scale
    (
    GNC_VECTOR3F vector,
    float scalar
    );

float gnc_vector3_dot
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    );

GNC_VECTOR3F gnc_vector3_cross
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    );

#ifdef __cplusplus
}
#endif

#endif /* GNC_VECTOR3_H */
