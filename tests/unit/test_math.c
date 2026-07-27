#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "gnc/quaternion.h"
#include "gnc/vector3.h"

#define TEST_TOLERANCE 1.0e-5f

static unsigned int passes;
static unsigned int failures;

static void check_true
    (
    bool condition,
    const char *name
    )
{
if ( condition )
    {
    passes++;
    }
else
    {
    failures++;
    printf("FAIL: %s\n", name);
    }
}

static bool nearly_equal
    (
    float actual,
    float expected
    )
{
return fabsf(actual - expected) <= TEST_TOLERANCE;

} /* nearly_equal */

static void check_vector
    (
    GNC_VECTOR3F actual,
    GNC_VECTOR3F expected,
    const char *name_x,
    const char *name_y,
    const char *name_z
    )
{
check_true(nearly_equal(actual.x, expected.x), name_x);
check_true(nearly_equal(actual.y, expected.y), name_y);
check_true(nearly_equal(actual.z, expected.z), name_z);

} /* check_vector */

static void test_vector_operations
    (
    void
    )
{
GNC_VECTOR3F a = { 1.0f, -2.0f, 3.0f };
GNC_VECTOR3F b = { 4.0f, 5.0f, -6.0f };
GNC_VECTOR3F result;
GNC_VECTOR3F expected;

check_true(gnc_vector3_is_finite(a), "finite vector accepted");
check_true
    (
    !gnc_vector3_is_finite((GNC_VECTOR3F){ NAN, 0.0f, 0.0f }),
    "non-finite vector rejected"
    );

result = gnc_vector3_add(a, b);
expected = (GNC_VECTOR3F){ 5.0f, 3.0f, -3.0f };
check_vector
    (
    result,
    expected,
    "vector add x",
    "vector add y",
    "vector add z"
    );

result = gnc_vector3_subtract(a, b);
expected = (GNC_VECTOR3F){ -3.0f, -7.0f, 9.0f };
check_vector
    (
    result,
    expected,
    "vector subtract x",
    "vector subtract y",
    "vector subtract z"
    );

result = gnc_vector3_scale(a, 2.0f);
expected = (GNC_VECTOR3F){ 2.0f, -4.0f, 6.0f };
check_vector
    (
    result,
    expected,
    "vector scale x",
    "vector scale y",
    "vector scale z"
    );

check_true(nearly_equal(gnc_vector3_dot(a, b), -24.0f), "vector dot");

result = gnc_vector3_cross
    (
    (GNC_VECTOR3F){ 1.0f, 0.0f, 0.0f },
    (GNC_VECTOR3F){ 0.0f, 1.0f, 0.0f }
    );
expected = (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f };
check_vector
    (
    result,
    expected,
    "cross product x",
    "cross product y",
    "cross product z"
    );

check_true
    (
    nearly_equal
        (
        gnc_vector3_magnitude((GNC_VECTOR3F){ 3.0f, 4.0f, 0.0f }),
        5.0f
        ),
    "vector magnitude"
    );

} /* test_vector_operations */

static void test_vector_normalization
    (
    void
    )
{
GNC_VECTOR3F vector = { 3.0f, 4.0f, 0.0f };
GNC_VECTOR3F zero = { 0.0f, 0.0f, 0.0f };
GNC_VECTOR3F nonfinite = { INFINITY, 0.0f, 0.0f };

check_true(gnc_vector3_normalize(&vector), "vector normalize succeeds");
check_true(nearly_equal(vector.x, 0.6f), "vector normalize x");
check_true(nearly_equal(vector.y, 0.8f), "vector normalize y");
check_true(nearly_equal(vector.z, 0.0f), "vector normalize z");
check_true
    (
    nearly_equal(gnc_vector3_magnitude(vector), 1.0f),
    "normalized vector magnitude"
    );

check_true(!gnc_vector3_normalize(&zero), "zero vector rejected");
check_true
    (
    !gnc_vector3_normalize(&nonfinite),
    "non-finite vector normalization rejected"
    );
check_true(!gnc_vector3_normalize(NULL), "null vector rejected");

} /* test_vector_normalization */

static void test_quaternion_operations
    (
    void
    )
{
GNC_QUATERNION identity = gnc_quaternion_identity();
GNC_QUATERNION q = { 1.0f, 2.0f, 3.0f, 4.0f };
GNC_QUATERNION result;

check_true(nearly_equal(identity.w, 1.0f), "identity quaternion w");
check_true(nearly_equal(identity.x, 0.0f), "identity quaternion x");
check_true(nearly_equal(identity.y, 0.0f), "identity quaternion y");
check_true(nearly_equal(identity.z, 0.0f), "identity quaternion z");

check_true(gnc_quaternion_is_finite(q), "finite quaternion accepted");
check_true
    (
    !gnc_quaternion_is_finite
        (
        (GNC_QUATERNION){ NAN, 0.0f, 0.0f, 0.0f }
        ),
    "non-finite quaternion rejected"
    );

result = gnc_quaternion_multiply(identity, q);
check_true(nearly_equal(result.w, q.w), "left identity w");
check_true(nearly_equal(result.x, q.x), "left identity x");
check_true(nearly_equal(result.y, q.y), "left identity y");
check_true(nearly_equal(result.z, q.z), "left identity z");

result = gnc_quaternion_multiply(q, identity);
check_true(nearly_equal(result.w, q.w), "right identity w");
check_true(nearly_equal(result.x, q.x), "right identity x");
check_true(nearly_equal(result.y, q.y), "right identity y");
check_true(nearly_equal(result.z, q.z), "right identity z");

result = gnc_quaternion_conjugate(q);
check_true(nearly_equal(result.w, 1.0f), "conjugate w");
check_true(nearly_equal(result.x, -2.0f), "conjugate x");
check_true(nearly_equal(result.y, -3.0f), "conjugate y");
check_true(nearly_equal(result.z, -4.0f), "conjugate z");

result = gnc_quaternion_add(q, identity);
check_true(nearly_equal(result.w, 2.0f), "quaternion add w");
check_true(nearly_equal(result.x, 2.0f), "quaternion add x");
check_true(nearly_equal(result.y, 3.0f), "quaternion add y");
check_true(nearly_equal(result.z, 4.0f), "quaternion add z");

result = gnc_quaternion_scale(q, 0.5f);
check_true(nearly_equal(result.w, 0.5f), "quaternion scale w");
check_true(nearly_equal(result.x, 1.0f), "quaternion scale x");
check_true(nearly_equal(result.y, 1.5f), "quaternion scale y");
check_true(nearly_equal(result.z, 2.0f), "quaternion scale z");

check_true(nearly_equal(gnc_quaternion_dot(q, identity), 1.0f), "quaternion dot");
check_true
    (
    nearly_equal(gnc_quaternion_norm((GNC_QUATERNION){ 2.0f, 0.0f, 0.0f, 0.0f }), 2.0f),
    "quaternion norm"
    );

} /* test_quaternion_operations */

static void test_quaternion_normalization
    (
    void
    )
{
GNC_QUATERNION quaternion = { 2.0f, 0.0f, 0.0f, 0.0f };
GNC_QUATERNION zero = { 0.0f, 0.0f, 0.0f, 0.0f };
GNC_QUATERNION nonfinite = { INFINITY, 0.0f, 0.0f, 0.0f };

check_true
    (
    gnc_quaternion_normalize(&quaternion),
    "quaternion normalize succeeds"
    );
check_true(nearly_equal(quaternion.w, 1.0f), "normalized quaternion w");
check_true(nearly_equal(quaternion.x, 0.0f), "normalized quaternion x");
check_true(nearly_equal(quaternion.y, 0.0f), "normalized quaternion y");
check_true(nearly_equal(quaternion.z, 0.0f), "normalized quaternion z");
check_true
    (
    nearly_equal(gnc_quaternion_norm(quaternion), 1.0f),
    "normalized quaternion norm"
    );

check_true(!gnc_quaternion_normalize(&zero), "zero quaternion rejected");
check_true
    (
    !gnc_quaternion_normalize(&nonfinite),
    "non-finite quaternion normalization rejected"
    );
check_true(!gnc_quaternion_normalize(NULL), "null quaternion rejected");

} /* test_quaternion_normalization */

static void test_identity_rotation
    (
    void
    )
{
GNC_QUATERNION attitude = gnc_quaternion_identity();
GNC_VECTOR3F input = { 1.0f, -2.0f, 3.0f };
GNC_VECTOR3F output = { 0.0f, 0.0f, 0.0f };

check_true
    (
    gnc_rotate_body_to_world(attitude, input, &output),
    "identity rotation succeeds"
    );

check_vector
    (
    output,
    input,
    "identity rotation x",
    "identity rotation y",
    "identity rotation z"
    );

} /* test_identity_rotation */

static void test_axis_rotations
    (
    void
    )
{
const float pi = 3.14159265358979323846f;
GNC_QUATERNION attitude;
GNC_VECTOR3F output = { 0.0f, 0.0f, 0.0f };

attitude = gnc_quaternion_from_euler_zyx(0.0f, 0.0f, pi * 0.5f);
check_true
    (
    gnc_rotate_body_to_world
        (
        attitude,
        (GNC_VECTOR3F){ 0.0f, 1.0f, 0.0f },
        &output
        ),
    "90 degree roll rotation succeeds"
    );
check_vector
    (
    output,
    (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
    "90 degree roll x",
    "90 degree roll y",
    "90 degree roll z"
    );

attitude = gnc_quaternion_from_euler_zyx(0.0f, pi * 0.5f, 0.0f);
check_true
    (
    gnc_rotate_body_to_world
        (
        attitude,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
        &output
        ),
    "90 degree pitch rotation succeeds"
    );
check_vector
    (
    output,
    (GNC_VECTOR3F){ 1.0f, 0.0f, 0.0f },
    "90 degree pitch x",
    "90 degree pitch y",
    "90 degree pitch z"
    );

attitude = gnc_quaternion_from_euler_zyx(pi * 0.5f, 0.0f, 0.0f);
check_true
    (
    gnc_rotate_body_to_world
        (
        attitude,
        (GNC_VECTOR3F){ 1.0f, 0.0f, 0.0f },
        &output
        ),
    "90 degree yaw rotation succeeds"
    );
check_vector
    (
    output,
    (GNC_VECTOR3F){ 0.0f, 1.0f, 0.0f },
    "90 degree yaw x",
    "90 degree yaw y",
    "90 degree yaw z"
    );

} /* test_axis_rotations */

static void test_round_trip_and_magnitude
    (
    void
    )
{
GNC_QUATERNION attitude;
GNC_VECTOR3F body = { 0.4f, -0.8f, 1.2f };
GNC_VECTOR3F world = { 0.0f, 0.0f, 0.0f };
GNC_VECTOR3F recovered = { 0.0f, 0.0f, 0.0f };
float original_magnitude;

attitude = gnc_quaternion_from_euler_zyx(0.5f, -0.3f, 0.2f);
original_magnitude = gnc_vector3_magnitude(body);

check_true
    (
    gnc_rotate_body_to_world(attitude, body, &world),
    "round trip body to world succeeds"
    );

check_true
    (
    nearly_equal(gnc_vector3_magnitude(world), original_magnitude),
    "rotation preserves magnitude"
    );

check_true
    (
    gnc_rotate_world_to_body(attitude, world, &recovered),
    "round trip world to body succeeds"
    );

check_vector
    (
    recovered,
    body,
    "round trip x",
    "round trip y",
    "round trip z"
    );

} /* test_round_trip_and_magnitude */

static void test_rotation_validation
    (
    void
    )
{
GNC_QUATERNION nonunit = { 2.0f, 0.0f, 0.0f, 0.0f };
GNC_QUATERNION zero = { 0.0f, 0.0f, 0.0f, 0.0f };
GNC_VECTOR3F input = { 1.0f, 2.0f, 3.0f };
GNC_VECTOR3F output = { 0.0f, 0.0f, 0.0f };

check_true
    (
    gnc_rotate_body_to_world(nonunit, input, &output),
    "rotation normalizes valid non-unit attitude"
    );
check_vector
    (
    output,
    input,
    "non-unit identity rotation x",
    "non-unit identity rotation y",
    "non-unit identity rotation z"
    );

check_true
    (
    !gnc_rotate_body_to_world(zero, input, &output),
    "zero attitude rotation rejected"
    );
check_true
    (
    !gnc_rotate_body_to_world
        (
        gnc_quaternion_identity(),
        input,
        NULL
        ),
    "null body-to-world output rejected"
    );
check_true
    (
    !gnc_rotate_world_to_body
        (
        gnc_quaternion_identity(),
        input,
        NULL
        ),
    "null world-to-body output rejected"
    );

} /* test_rotation_validation */

int main
    (
    void
    )
{
test_vector_operations();
test_vector_normalization();
test_quaternion_operations();
test_quaternion_normalization();
test_identity_rotation();
test_axis_rotations();
test_round_trip_and_magnitude();
test_rotation_validation();

printf("\nPasses: %u\n", passes);
printf("Fails:  %u\n", failures);

return failures == 0U ? 0 : 1;

} /* main */
