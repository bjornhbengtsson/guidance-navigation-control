#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "gnc/mekf.h"

#define TEST_TOLERANCE 1.0e-6f

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

static GNC_MEKF_CONFIG valid_config
    (
    void
    )
{
GNC_MEKF_CONFIG config;

config.initial_attitude_std_rad =
    (GNC_VECTOR3F){ 0.10f, 0.20f, 0.30f };

config.initial_gyro_bias_std_rad_s =
    (GNC_VECTOR3F){ 0.01f, 0.02f, 0.03f };

config.gyro_noise_density_rad_s_sqrt_hz = 0.004f;
config.gyro_bias_random_walk_rad_s2_sqrt_hz = 0.0001f;
config.maximum_delta_time_s = 0.10f;

return config;

} /* valid_config */

static void test_initialization_success
    (
    void
    )
{
GNC_MEKF_FILTER filter;
GNC_MEKF_CONFIG config = valid_config();
GNC_QUATERNION initial_attitude = { 2.0f, 0.0f, 0.0f, 0.0f };
GNC_VECTOR3F initial_bias = { 0.01f, -0.02f, 0.03f };

check_true
    (
    gnc_mekf_init
        (
        &filter,
        initial_attitude,
        initial_bias,
        &config
        ),
    "MEKF initialization succeeds"
    );

check_true
    (
    nearly_equal(filter.attitude_body_to_world.w, 1.0f),
    "initial attitude normalized w"
    );
check_true
    (
    nearly_equal(filter.attitude_body_to_world.x, 0.0f),
    "initial attitude normalized x"
    );
check_true
    (
    nearly_equal(filter.attitude_body_to_world.y, 0.0f),
    "initial attitude normalized y"
    );
check_true
    (
    nearly_equal(filter.attitude_body_to_world.z, 0.0f),
    "initial attitude normalized z"
    );

check_true
    (
    nearly_equal(filter.gyro_bias_body_rad_s.x, initial_bias.x),
    "initial gyro bias x stored"
    );
check_true
    (
    nearly_equal(filter.gyro_bias_body_rad_s.y, initial_bias.y),
    "initial gyro bias y stored"
    );
check_true
    (
    nearly_equal(filter.gyro_bias_body_rad_s.z, initial_bias.z),
    "initial gyro bias z stored"
    );

check_true
    (
    nearly_equal(filter.config.maximum_delta_time_s, 0.10f),
    "configuration stored"
    );

} /* test_initialization_success */

static void test_covariance_initialization
    (
    void
    )
{
unsigned int row;
unsigned int column;
GNC_MEKF_FILTER filter;
GNC_MEKF_CONFIG config = valid_config();
float expected_diagonal[GNC_MEKF_ERROR_STATE_DIM] =
    {
    0.01f,
    0.04f,
    0.09f,
    0.0001f,
    0.0004f,
    0.0009f
    };

check_true
    (
    gnc_mekf_init
        (
        &filter,
        gnc_quaternion_identity(),
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
        &config
        ),
    "covariance initialization succeeds"
    );

for ( row = 0U; row < GNC_MEKF_ERROR_STATE_DIM; row++ )
    {
    for ( column = 0U;
          column < GNC_MEKF_ERROR_STATE_DIM;
          column++ )
        {
        float expected =
            row == column ?
            expected_diagonal[row] :
            0.0f;

        check_true
            (
            nearly_equal
                (
                filter.covariance[row][column],
                expected
                ),
            "covariance element initialized"
            );
        }
    }

} /* test_covariance_initialization */

static void test_invalid_arguments
    (
    void
    )
{
GNC_MEKF_FILTER filter;
GNC_MEKF_CONFIG config = valid_config();
GNC_QUATERNION identity = gnc_quaternion_identity();
GNC_VECTOR3F zero_bias = { 0.0f, 0.0f, 0.0f };

check_true
    (
    !gnc_mekf_init(NULL, identity, zero_bias, &config),
    "null filter rejected"
    );

check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, NULL),
    "null config rejected"
    );

check_true
    (
    !gnc_mekf_init
        (
        &filter,
        (GNC_QUATERNION){ 0.0f, 0.0f, 0.0f, 0.0f },
        zero_bias,
        &config
        ),
    "zero quaternion rejected"
    );

check_true
    (
    !gnc_mekf_init
        (
        &filter,
        (GNC_QUATERNION){ NAN, 0.0f, 0.0f, 0.0f },
        zero_bias,
        &config
        ),
    "non-finite quaternion rejected"
    );

check_true
    (
    !gnc_mekf_init
        (
        &filter,
        identity,
        (GNC_VECTOR3F){ INFINITY, 0.0f, 0.0f },
        &config
        ),
    "non-finite gyro bias rejected"
    );

} /* test_invalid_arguments */

static void test_invalid_configuration
    (
    void
    )
{
GNC_MEKF_FILTER filter;
GNC_MEKF_CONFIG config;
GNC_QUATERNION identity = gnc_quaternion_identity();
GNC_VECTOR3F zero_bias = { 0.0f, 0.0f, 0.0f };

config = valid_config();
config.initial_attitude_std_rad.x = -0.1f;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "negative attitude standard deviation rejected"
    );

config = valid_config();
config.initial_gyro_bias_std_rad_s.y = NAN;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "non-finite bias standard deviation rejected"
    );

config = valid_config();
config.gyro_noise_density_rad_s_sqrt_hz = -0.1f;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "negative gyro noise rejected"
    );

config = valid_config();
config.gyro_bias_random_walk_rad_s2_sqrt_hz = INFINITY;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "non-finite bias random walk rejected"
    );

config = valid_config();
config.maximum_delta_time_s = 0.0f;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "zero maximum delta time rejected"
    );

config = valid_config();
config.maximum_delta_time_s = -0.1f;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "negative maximum delta time rejected"
    );

config = valid_config();
config.maximum_delta_time_s = NAN;
check_true
    (
    !gnc_mekf_init(&filter, identity, zero_bias, &config),
    "non-finite maximum delta time rejected"
    );

} /* test_invalid_configuration */

static void test_failed_init_is_transactional
    (
    void
    )
{
GNC_MEKF_FILTER filter;
GNC_MEKF_FILTER before;
GNC_MEKF_CONFIG config = valid_config();

memset(&filter, 0x5A, sizeof(filter));
before = filter;

check_true
    (
    !gnc_mekf_init
        (
        &filter,
        (GNC_QUATERNION){ 0.0f, 0.0f, 0.0f, 0.0f },
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
        &config
        ),
    "invalid transactional initialization rejected"
    );

check_true
    (
    memcmp(&filter, &before, sizeof(filter)) == 0,
    "failed initialization leaves filter unchanged"
    );

} /* test_failed_init_is_transactional */

int main
    (
    void
    )
{
test_initialization_success();
test_covariance_initialization();
test_invalid_arguments();
test_invalid_configuration();
test_failed_init_is_transactional();

printf("\nPasses: %u\n", passes);
printf("Fails:  %u\n", failures);

return failures == 0U ? 0 : 1;

} /* main */
