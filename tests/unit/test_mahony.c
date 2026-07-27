#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "gnc/mahony.h"

#define TEST_PI           3.14159265358979323846f
#define TEST_TOLERANCE    1.0e-3f

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

static bool quaternion_nearly_equal
    (
    GNC_QUATERNION actual,
    GNC_QUATERNION expected
    )
{
return
    (
    nearly_equal(actual.w, expected.w) &&
    nearly_equal(actual.x, expected.x) &&
    nearly_equal(actual.y, expected.y) &&
    nearly_equal(actual.z, expected.z)
    );

} /* quaternion_nearly_equal */

static void test_default_config
    (
    void
    )
{
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();

check_true(nearly_equal(config.proportional_gain, 1.0f), "default Kp");
check_true(nearly_equal(config.integral_gain, 0.0f), "default Ki");
check_true
    (
    config.accel_min_magnitude_mps2 > 0.0f,
    "default accel minimum positive"
    );
check_true
    (
    config.accel_max_magnitude_mps2 >
        config.accel_min_magnitude_mps2,
    "default accel maximum above minimum"
    );
check_true
    (
    nearly_equal(config.integral_limit_rad_s, 0.25f),
    "default integral limit"
    );

} /* test_default_config */

static void test_initialization
    (
    void
    )
{
GNC_MAHONY_FILTER filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_QUATERNION nonunit = { 2.0f, 0.0f, 0.0f, 0.0f };
GNC_QUATERNION zero = { 0.0f, 0.0f, 0.0f, 0.0f };

check_true
    (
    gnc_mahony_init(&filter, nonunit, &config),
    "Mahony initialization succeeds"
    );
check_true
    (
    quaternion_nearly_equal
        (
        filter.attitude_body_to_world,
        gnc_quaternion_identity()
        ),
    "initial attitude normalized"
    );
check_true
    (
    nearly_equal(filter.integral_error_rad_s.x, 0.0f) &&
    nearly_equal(filter.integral_error_rad_s.y, 0.0f) &&
    nearly_equal(filter.integral_error_rad_s.z, 0.0f),
    "integral state cleared"
    );

check_true
    (
    !gnc_mahony_init(NULL, nonunit, &config),
    "null filter rejected"
    );
check_true
    (
    !gnc_mahony_init(&filter, zero, &config),
    "zero quaternion rejected"
    );
check_true
    (
    !gnc_mahony_init
        (
        &filter,
        (GNC_QUATERNION){ NAN, 0.0f, 0.0f, 0.0f },
        &config
        ),
    "non-finite quaternion rejected"
    );
check_true
    (
    !gnc_mahony_init(&filter, nonunit, NULL),
    "null config rejected"
    );

config.proportional_gain = -1.0f;
check_true
    (
    !gnc_mahony_init(&filter, nonunit, &config),
    "negative proportional gain rejected"
    );

config = gnc_mahony_default_config();
config.integral_gain = -1.0f;
check_true
    (
    !gnc_mahony_init(&filter, nonunit, &config),
    "negative integral gain rejected"
    );

config = gnc_mahony_default_config();
config.accel_max_magnitude_mps2 =
    config.accel_min_magnitude_mps2 - 1.0f;
check_true
    (
    !gnc_mahony_init(&filter, nonunit, &config),
    "invalid acceleration range rejected"
    );

} /* test_initialization */

static void test_zero_rate
    (
    void
    )
{
GNC_MAHONY_FILTER filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_QUATERNION identity = gnc_quaternion_identity();

check_true
    (
    gnc_mahony_init(&filter, identity, &config),
    "zero-rate filter initialized"
    );
check_true
    (
    gnc_mahony_update_gyro
        (
        &filter,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
        0.01f
        ),
    "zero-rate update succeeds"
    );
check_true
    (
    quaternion_nearly_equal(filter.attitude_body_to_world, identity),
    "zero-rate update preserves attitude"
    );

} /* test_zero_rate */

static void test_positive_yaw
    (
    void
    )
{
int index;
GNC_MAHONY_FILTER filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_VECTOR3F world;
GNC_QUATERNION expected;

config.proportional_gain = 0.0f;

check_true
    (
    gnc_mahony_init
        (
        &filter,
        gnc_quaternion_identity(),
        &config
        ),
    "yaw filter initialized"
    );

for ( index = 0; index < 1000; index++ )
    {
    check_true
        (
        gnc_mahony_update_gyro
            (
            &filter,
            (GNC_VECTOR3F){ 0.0f, 0.0f, 0.5f * TEST_PI },
            0.001f
            ),
        "positive yaw propagation succeeds"
        );
    }

expected = gnc_quaternion_from_euler_zyx(0.5f * TEST_PI, 0.0f, 0.0f);

check_true
    (
    quaternion_nearly_equal(filter.attitude_body_to_world, expected),
    "positive yaw quaternion"
    );

check_true
    (
    gnc_rotate_body_to_world
        (
        filter.attitude_body_to_world,
        (GNC_VECTOR3F){ 1.0f, 0.0f, 0.0f },
        &world
        ),
    "positive yaw vector rotation succeeds"
    );
check_true(nearly_equal(world.x, 0.0f), "positive yaw world x");
check_true(nearly_equal(world.y, 1.0f), "positive yaw world y");
check_true(nearly_equal(world.z, 0.0f), "positive yaw world z");

} /* test_positive_yaw */

static void test_invalid_gyro_update
    (
    void
    )
{
GNC_MAHONY_FILTER filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_QUATERNION before;

check_true
    (
    gnc_mahony_init
        (
        &filter,
        gnc_quaternion_identity(),
        &config
        ),
    "invalid-update filter initialized"
    );

before = filter.attitude_body_to_world;

check_true
    (
    !gnc_mahony_update_gyro
        (
        &filter,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
        0.0f
        ),
    "zero delta time rejected"
    );
check_true
    (
    !gnc_mahony_update_gyro
        (
        &filter,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
        -0.01f
        ),
    "negative delta time rejected"
    );
check_true
    (
    !gnc_mahony_update_gyro
        (
        &filter,
        (GNC_VECTOR3F){ NAN, 0.0f, 0.0f },
        0.01f
        ),
    "non-finite gyro rejected"
    );
check_true
    (
    quaternion_nearly_equal(filter.attitude_body_to_world, before),
    "failed gyro update preserves state"
    );

} /* test_invalid_gyro_update */

static void test_aligned_gravity
    (
    void
    )
{
int index;
GNC_MAHONY_FILTER filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();

check_true
    (
    gnc_mahony_init
        (
        &filter,
        gnc_quaternion_identity(),
        &config
        ),
    "aligned-gravity filter initialized"
    );

for ( index = 0; index < 1000; index++ )
    {
    check_true
        (
        gnc_mahony_update_imu
            (
            &filter,
            (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
            (GNC_VECTOR3F){ 0.0f, 0.0f, 9.8f },
            0.001f,
            true
            ),
        "aligned-gravity update succeeds"
        );
    }

check_true
    (
    quaternion_nearly_equal
        (
        filter.attitude_body_to_world,
        gnc_quaternion_identity()
        ),
    "aligned gravity preserves identity"
    );

} /* test_aligned_gravity */

static void test_roll_and_pitch_convergence
    (
    void
    )
{
int index;
GNC_MAHONY_FILTER roll_filter;
GNC_MAHONY_FILTER pitch_filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_VECTOR3F roll_world_z_initial;
GNC_VECTOR3F roll_world_z_final;
GNC_VECTOR3F pitch_world_z_initial;
GNC_VECTOR3F pitch_world_z_final;

check_true
    (
    gnc_mahony_init
        (
        &roll_filter,
        gnc_quaternion_from_euler_zyx
            (
            0.0f,
            0.0f,
            10.0f * TEST_PI / 180.0f
            ),
        &config
        ),
    "roll convergence filter initialized"
    );

check_true
    (
    gnc_mahony_init
        (
        &pitch_filter,
        gnc_quaternion_from_euler_zyx
            (
            0.0f,
            -10.0f * TEST_PI / 180.0f,
            0.0f
            ),
        &config
        ),
    "pitch convergence filter initialized"
    );

check_true
    (
    gnc_rotate_body_to_world
        (
        roll_filter.attitude_body_to_world,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
        &roll_world_z_initial
        ),
    "initial roll vector succeeds"
    );

check_true
    (
    gnc_rotate_body_to_world
        (
        pitch_filter.attitude_body_to_world,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
        &pitch_world_z_initial
        ),
    "initial pitch vector succeeds"
    );

for ( index = 0; index < 5000; index++ )
    {
    check_true
        (
        gnc_mahony_update_imu
            (
            &roll_filter,
            (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
            (GNC_VECTOR3F){ 0.0f, 0.0f, 9.8f },
            0.001f,
            true
            ),
        "roll convergence update succeeds"
        );

    check_true
        (
        gnc_mahony_update_imu
            (
            &pitch_filter,
            (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
            (GNC_VECTOR3F){ 0.0f, 0.0f, 9.8f },
            0.001f,
            true
            ),
        "pitch convergence update succeeds"
        );
    }

check_true
    (
    gnc_rotate_body_to_world
        (
        roll_filter.attitude_body_to_world,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
        &roll_world_z_final
        ),
    "final roll vector succeeds"
    );

check_true
    (
    gnc_rotate_body_to_world
        (
        pitch_filter.attitude_body_to_world,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 1.0f },
        &pitch_world_z_final
        ),
    "final pitch vector succeeds"
    );

check_true
    (
    roll_world_z_final.z > roll_world_z_initial.z,
    "roll error converges"
    );
check_true
    (
    pitch_world_z_final.z > pitch_world_z_initial.z,
    "pitch error converges"
    );

} /* test_roll_and_pitch_convergence */

static void test_accelerometer_fallback
    (
    void
    )
{
GNC_MAHONY_FILTER gyro_only;
GNC_MAHONY_FILTER disabled;
GNC_MAHONY_FILTER invalid_low;
GNC_MAHONY_FILTER invalid_high;
GNC_MAHONY_FILTER invalid_nonfinite;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_VECTOR3F gyro = { 0.1f, -0.2f, 0.3f };
GNC_QUATERNION identity = gnc_quaternion_identity();

check_true(gnc_mahony_init(&gyro_only, identity, &config), "gyro-only initialized");
check_true(gnc_mahony_init(&disabled, identity, &config), "disabled initialized");
check_true(gnc_mahony_init(&invalid_low, identity, &config), "low initialized");
check_true(gnc_mahony_init(&invalid_high, identity, &config), "high initialized");
check_true(gnc_mahony_init(&invalid_nonfinite, identity, &config), "nonfinite initialized");

check_true
    (
    gnc_mahony_update_gyro(&gyro_only, gyro, 0.01f),
    "gyro-only update succeeds"
    );
check_true
    (
    gnc_mahony_update_imu
        (
        &disabled,
        gyro,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 9.8f },
        0.01f,
        false
        ),
    "disabled accel update succeeds"
    );
check_true
    (
    gnc_mahony_update_imu
        (
        &invalid_low,
        gyro,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.01f },
        0.01f,
        true
        ),
    "low accel falls back"
    );
check_true
    (
    gnc_mahony_update_imu
        (
        &invalid_high,
        gyro,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 100.0f },
        0.01f,
        true
        ),
    "high accel falls back"
    );
check_true
    (
    gnc_mahony_update_imu
        (
        &invalid_nonfinite,
        gyro,
        (GNC_VECTOR3F){ NAN, 0.0f, 0.0f },
        0.01f,
        true
        ),
    "non-finite accel falls back"
    );

check_true
    (
    quaternion_nearly_equal
        (
        disabled.attitude_body_to_world,
        gyro_only.attitude_body_to_world
        ),
    "disabled accel matches gyro-only"
    );
check_true
    (
    quaternion_nearly_equal
        (
        invalid_low.attitude_body_to_world,
        gyro_only.attitude_body_to_world
        ),
    "low accel matches gyro-only"
    );
check_true
    (
    quaternion_nearly_equal
        (
        invalid_high.attitude_body_to_world,
        gyro_only.attitude_body_to_world
        ),
    "high accel matches gyro-only"
    );
check_true
    (
    quaternion_nearly_equal
        (
        invalid_nonfinite.attitude_body_to_world,
        gyro_only.attitude_body_to_world
        ),
    "non-finite accel matches gyro-only"
    );

} /* test_accelerometer_fallback */

static void test_integral_correction_and_limit
    (
    void
    )
{
int index;
GNC_MAHONY_FILTER filter;
GNC_MAHONY_CONFIG config = gnc_mahony_default_config();
GNC_VECTOR3F before_freeze;

config.proportional_gain = 0.0f;
config.integral_gain = 10.0f;
config.integral_limit_rad_s = 0.02f;

check_true
    (
    gnc_mahony_init
        (
        &filter,
        gnc_quaternion_from_euler_zyx
            (
            0.0f,
            0.0f,
            30.0f * TEST_PI / 180.0f
            ),
        &config
        ),
    "integral filter initialized"
    );

for ( index = 0; index < 1000; index++ )
    {
    check_true
        (
        gnc_mahony_update_imu
            (
            &filter,
            (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
            (GNC_VECTOR3F){ 0.0f, 0.0f, 9.8f },
            0.001f,
            true
            ),
        "integral update succeeds"
        );
    }

check_true
    (
    fabsf(filter.integral_error_rad_s.x) <=
        config.integral_limit_rad_s + TEST_TOLERANCE,
    "integral x limited"
    );
check_true
    (
    fabsf(filter.integral_error_rad_s.y) <=
        config.integral_limit_rad_s + TEST_TOLERANCE,
    "integral y limited"
    );
check_true
    (
    fabsf(filter.integral_error_rad_s.z) <=
        config.integral_limit_rad_s + TEST_TOLERANCE,
    "integral z limited"
    );

before_freeze = filter.integral_error_rad_s;

check_true
    (
    gnc_mahony_update_imu
        (
        &filter,
        (GNC_VECTOR3F){ 0.0f, 0.0f, 0.0f },
        (GNC_VECTOR3F){ 0.0f, 0.0f, 9.8f },
        0.001f,
        false
        ),
    "integral freeze update succeeds"
    );

check_true
    (
    nearly_equal
        (
        filter.integral_error_rad_s.x,
        before_freeze.x
        ) &&
    nearly_equal
        (
        filter.integral_error_rad_s.y,
        before_freeze.y
        ) &&
    nearly_equal
        (
        filter.integral_error_rad_s.z,
        before_freeze.z
        ),
    "integral freezes while accel disabled"
    );

} /* test_integral_correction_and_limit */

int main
    (
    void
    )
{
test_default_config();
test_initialization();
test_zero_rate();
test_positive_yaw();
test_invalid_gyro_update();
test_aligned_gravity();
test_roll_and_pitch_convergence();
test_accelerometer_fallback();
test_integral_correction_and_limit();

printf("\nPasses: %u\n", passes);
printf("Fails:  %u\n", failures);

return failures == 0U ? 0 : 1;

} /* main */
