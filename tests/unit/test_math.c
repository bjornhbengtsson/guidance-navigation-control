#include <math.h>
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

check_true(nearly_equal(output.x, input.x), "identity rotation x");
check_true(nearly_equal(output.y, input.y), "identity rotation y");
check_true(nearly_equal(output.z, input.z), "identity rotation z");

} /* test_identity_rotation */

static void test_yaw_90_degrees
    (
    void
    )
{
const float pi = 3.14159265358979323846f;
GNC_QUATERNION attitude;
GNC_VECTOR3F body_x = { 1.0f, 0.0f, 0.0f };
GNC_VECTOR3F world = { 0.0f, 0.0f, 0.0f };

attitude = gnc_quaternion_from_euler_zyx(pi * 0.5f, 0.0f, 0.0f);

check_true
    (
    gnc_rotate_body_to_world(attitude, body_x, &world),
    "90 degree yaw rotation succeeds"
    );

check_true(nearly_equal(world.x, 0.0f), "90 degree yaw x");
check_true(nearly_equal(world.y, 1.0f), "90 degree yaw y");
check_true(nearly_equal(world.z, 0.0f), "90 degree yaw z");

} /* test_yaw_90_degrees */

static void test_round_trip
    (
    void
    )
{
GNC_QUATERNION attitude;
GNC_VECTOR3F body = { 0.4f, -0.8f, 1.2f };
GNC_VECTOR3F world = { 0.0f, 0.0f, 0.0f };
GNC_VECTOR3F recovered = { 0.0f, 0.0f, 0.0f };

attitude = gnc_quaternion_from_euler_zyx(0.5f, -0.3f, 0.2f);

check_true
    (
    gnc_rotate_body_to_world(attitude, body, &world),
    "round trip body to world succeeds"
    );

check_true
    (
    gnc_rotate_world_to_body(attitude, world, &recovered),
    "round trip world to body succeeds"
    );

check_true(nearly_equal(recovered.x, body.x), "round trip x");
check_true(nearly_equal(recovered.y, body.y), "round trip y");
check_true(nearly_equal(recovered.z, body.z), "round trip z");

} /* test_round_trip */

static void test_normalization_rejects_zero
    (
    void
    )
{
GNC_QUATERNION zero = { 0.0f, 0.0f, 0.0f, 0.0f };

check_true
    (
    !gnc_quaternion_normalize(&zero),
    "zero quaternion normalization rejected"
    );

} /* test_normalization_rejects_zero */

static void test_vector_cross_product
    (
    void
    )
{
GNC_VECTOR3F x = { 1.0f, 0.0f, 0.0f };
GNC_VECTOR3F y = { 0.0f, 1.0f, 0.0f };
GNC_VECTOR3F result = gnc_vector3_cross(x, y);

check_true(nearly_equal(result.x, 0.0f), "cross product x");
check_true(nearly_equal(result.y, 0.0f), "cross product y");
check_true(nearly_equal(result.z, 1.0f), "cross product z");

} /* test_vector_cross_product */

int main
    (
    void
    )
{
test_identity_rotation();
test_yaw_90_degrees();
test_round_trip();
test_normalization_rejects_zero();
test_vector_cross_product();

printf("\nPasses: %u\n", passes);
printf("Fails:  %u\n", failures);

return failures == 0U ? 0 : 1;

} /* main */
