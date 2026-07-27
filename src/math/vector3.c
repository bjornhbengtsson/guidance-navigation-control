/*******************************************************************************
 *
 * FILE:
 *      vector3.c
 *
 * DESCRIPTION:
 *      Portable three-dimensional vector implementation.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************/

#include <math.h>
#include <stddef.h>

#include "gnc/vector3.h"

bool gnc_vector3_is_finite
    (
    GNC_VECTOR3F vector
    )
{
return
    (
    isfinite(vector.x) &&
    isfinite(vector.y) &&
    isfinite(vector.z)
    );

} /* gnc_vector3_is_finite */

float gnc_vector3_magnitude
    (
    GNC_VECTOR3F vector
    )
{
return sqrtf
    (
    vector.x * vector.x +
    vector.y * vector.y +
    vector.z * vector.z
    );

} /* gnc_vector3_magnitude */

bool gnc_vector3_normalize
    (
    GNC_VECTOR3F *vector
    )
{
float magnitude;

if ( vector == NULL ||
     !gnc_vector3_is_finite(*vector) )
    {
    return false;
    }

magnitude = gnc_vector3_magnitude(*vector);

if ( !isfinite(magnitude) ||
     magnitude <= 0.0f )
    {
    return false;
    }

vector->x /= magnitude;
vector->y /= magnitude;
vector->z /= magnitude;

return gnc_vector3_is_finite(*vector);

} /* gnc_vector3_normalize */

GNC_VECTOR3F gnc_vector3_add
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    )
{
GNC_VECTOR3F result;

result.x = a.x + b.x;
result.y = a.y + b.y;
result.z = a.z + b.z;

return result;

} /* gnc_vector3_add */

GNC_VECTOR3F gnc_vector3_subtract
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    )
{
GNC_VECTOR3F result;

result.x = a.x - b.x;
result.y = a.y - b.y;
result.z = a.z - b.z;

return result;

} /* gnc_vector3_subtract */

GNC_VECTOR3F gnc_vector3_scale
    (
    GNC_VECTOR3F vector,
    float scalar
    )
{
GNC_VECTOR3F result;

result.x = vector.x * scalar;
result.y = vector.y * scalar;
result.z = vector.z * scalar;

return result;

} /* gnc_vector3_scale */

float gnc_vector3_dot
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    )
{
return
    (
    a.x * b.x +
    a.y * b.y +
    a.z * b.z
    );

} /* gnc_vector3_dot */

GNC_VECTOR3F gnc_vector3_cross
    (
    GNC_VECTOR3F a,
    GNC_VECTOR3F b
    )
{
GNC_VECTOR3F result;

result.x = a.y * b.z - a.z * b.y;
result.y = a.z * b.x - a.x * b.z;
result.z = a.x * b.y - a.y * b.x;

return result;

} /* gnc_vector3_cross */
