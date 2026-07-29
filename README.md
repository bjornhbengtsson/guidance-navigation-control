# Guidance, Navigation, and Control

A portable C workspace for developing, testing, documenting, and validating Guidance, Navigation, and Control algorithms for rockets, spacecraft, simulation, and future embedded flight computers.

This repository builds on publicly shareable work developed for Sun Devil Rocketry while separating reusable algorithms from STM32 HAL, board-specific drivers, telemetry, and application logic.

## Current Progress

```text
IMU architecture and selection     ██████████  Complete
BMI270/BMM150 driver path          ██████████  Complete
Interrupt-driven sensor updates    ██████████  ~446 Hz achieved
Raw sensor conversion and units    ██████████  Complete
IMU axis and frame handling        ████████░░  Hardware validation ongoing
Euler-to-quaternion utilities      ██████████  Complete
Quaternion sensor-state pipeline   ██████████  Complete

Quaternion and frame math         ██████████  Complete
Gravity compensation              ██████████  Complete
World-frame velocity foundation   ███████░░░  Validation ongoing

Mahony attitude filter            ██████████  Complete
Mahony unit tests                 ██████████  20,141 passes
Mahony hardware validation        ██░░░░░░░░  In progress

MEKF initialization               ██████████  Complete
MEKF attitude prediction          ██████████  Complete
MEKF covariance prediction        ██████████  Complete
MEKF accelerometer correction     ██████████  Complete
MEKF unit tests                   ██████████  722 passes
MEKF hardware integration         ░░░░░░░░░░  Planned

Magnetometer fusion               ░░░░░░░░░░  Calibration required first
Navigation fusion                 █░░░░░░░░░  Early foundation
Guidance                          ░░░░░░░░░░  Planned
Control                           ░░░░░░░░░░  Planned
```

## Architecture

```text
Hardware Sensors
(IMU, Magnetometer, Barometer, GPS)
      │
      ▼
Platform Drivers
      │
      ▼
Platform Adapter
      │
      ▼
Normalized Sensor Sample
      │
      ▼
Attitude Estimation
  ├── Mahony
  └── MEKF
      │
      ▼
Navigation
      │
      ▼
Guidance
      │
      ▼
Control
      │
      ▼
Actuator Adapter
      │
      ▼
Physical Actuators
```

Simulation, recorded-data playback, and automated tests connect to the same portable estimator and navigation interfaces.

## Implemented Foundations

### Quaternion and Frame Mathematics

* Quaternion multiplication
* Quaternion normalization
* Quaternion conjugation
* Quaternion addition and scaling
* Body-to-world vector rotation
* World-to-body vector rotation
* Quaternion ordering: `[w, x, y, z]`
* Stored attitude convention: body-to-world

Body-to-world rotation:

```text
vector_world =
    attitude *
    vector_body *
    conjugate(attitude)
```

World-to-body rotation:

```text
vector_body =
    conjugate(attitude) *
    vector_world *
    attitude
```

### Gravity Compensation

The current navigation foundation:

1. Defines gravity in the world frame.
2. Rotates gravity into the body frame.
3. Removes gravity from the accelerometer measurement.
4. Rotates linear acceleration into the world frame.
5. Integrates world-frame acceleration into velocity.

```text
accelerometer measurement
          │
          ▼
predicted body-frame gravity
          │
          ▼
gravity-compensated body acceleration
          │
          ▼
world-frame linear acceleration
          │
          ▼
world-frame velocity
```

Pure inertial integration will drift over time. GPS, barometer, and other correction sources are planned for later navigation milestones.

## Mahony Attitude Filter

The Mahony filter is one of the attitude estimators.

Implemented capabilities:

* Gyroscope quaternion propagation
* Proportional accelerometer correction
* Integral gyro-bias correction
* Integral anti-windup
* Accelerometer magnitude validation
* Flight-state accelerometer gating
* Gyro-only fallback
* Quaternion normalization
* Host-side unit and diagnostic tests

Current SDR integration:

```text
sensor_body_state()
        │
        ├── converts gyro from deg/s to rad/s
        ├── calculates delta time
        ├── determines whether accelerometer correction is valid
        ▼
mahony_update_imu()
        │
        ▼
state_estimate->attitude
```

Latest recorded Mahony test result:

```text
Passes: 20141
Fails:  0
Result: PASS
```

Accelerometer correction constrains roll and pitch relative to gravity. Without a magnetometer, yaw remains unobservable from accelerometer measurements alone.

## Multiplicative Extended Kalman Filter

The repository includes a six-state attitude and gyro-bias MEKF.

### Error State

```text
delta_x =
[
    delta_theta_x
    delta_theta_y
    delta_theta_z
    delta_bias_x
    delta_bias_y
    delta_bias_z
]
```

The nominal state contains:

* Body-to-world attitude quaternion
* Three-axis gyro-bias estimate

The `6 × 6` covariance represents uncertainty in the local attitude error and gyro-bias error, not uncertainty in the four quaternion components directly.

### Complete MEKF Covariance Cycle
┌───────────────────────────────────────────────────────────────────────────┐
│ 1. INITIAL COVARIANCE                                                     │
│                                                                           │
│ Error state:                                                              │
│                                                                           │
│        δx = [ δθx  δθy  δθz  δbx  δby  δbz ]ᵀ                            │
│                                                                           │
│ Initial covariance:                                                       │
│                                                                           │
│             ┌ σ²θx    0      0      0      0      0   ┐                   │
│             │  0     σ²θy    0      0      0      0   │                   │
│             │  0      0     σ²θz    0      0      0   │                   │
│      P₀  =  │  0      0      0     σ²bx    0      0   │                   │
│             │  0      0      0      0     σ²by    0   │                   │
│             └  0      0      0      0      0     σ²bz ┘                   │
│                                                                           │
│             P₀ = diag(σ²θx, σ²θy, σ²θz, σ²bx, σ²by, σ²bz)                 │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 2. GYROSCOPE ATTITUDE PREDICTION                                         │
│                                                                           │
│ Correct the measured angular rate using the estimated gyro bias:          │
│                                                                           │
│             ω̂ = ωm - b̂g                                                  │
│                                                                           │
│ Integrate the corrected rate:                                             │
│                                                                           │
│             Δθ = ω̂ Δt                                                    │
│                                                                           │
│ Convert the incremental rotation into a quaternion:                       │
│                                                                           │
│             δq = [ cos(||Δθ||/2),                                         │
│                    (Δθ/||Δθ||) sin(||Δθ||/2) ]                            │
│                                                                           │
│ Propagate the nominal body-to-world attitude:                             │
│                                                                           │
│             q⁻ = normalize(q⁺ ⊗ δq)                                       │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 3. STATE-TRANSITION MATRIX                                                │
│                                                                           │
│ First-order six-state transition matrix:                                  │
│                                                                           │
│                    ┌                         ┐                            │
│                    │ I - [ω̂×]Δt      -IΔt  │                            │
│             Φ  =   │                         │                            │
│                    │     0              I    │                            │
│                    └                         ┘                            │
│                                                                           │
│ Expanded attitude block:                                                  │
│                                                                           │
│                      ┌ 1       ω̂zΔt   -ω̂yΔt ┐                            │
│   I - [ω̂×]Δt   =    │-ω̂zΔt   1        ω̂xΔt │                            │
│                      └ ω̂yΔt  -ω̂xΔt    1     ┘                            │
│                                                                           │
│ Expanded six-state matrix:                                                │
│                                                                           │
│        ┌ 1       ω̂zΔt  -ω̂yΔt  -Δt    0      0   ┐                       │
│        │-ω̂zΔt   1       ω̂xΔt    0    -Δt     0   │                       │
│        │ ω̂yΔt  -ω̂xΔt   1        0     0     -Δt  │                       │
│  Φ  =  │ 0       0       0        1     0      0   │                       │
│        │ 0       0       0        0     1      0   │                       │
│        └ 0       0       0        0     0      1   ┘                       │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 4. DISCRETE PROCESS-NOISE MATRIX                                          │
│                                                                           │
│ Gyroscope white-noise variance:                                           │
│                                                                           │
│             σ²g = gyro_noise_density²                                     │
│                                                                           │
│ Gyro-bias random-walk variance:                                           │
│                                                                           │
│             σ²b = bias_random_walk_density²                               │
│                                                                           │
│ Process-noise terms:                                                      │
│                                                                           │
│             Qθθ = σ²g Δt + σ²b Δt³/3                                     │
│             Qθb = -σ²b Δt²/2                                              │
│             Qbb = σ²b Δt                                                  │
│                                                                           │
│ Discrete process-noise matrix:                                            │
│                                                                           │
│                     ┌                         ┐                           │
│                     │ Qθθ I          Qθb I    │                           │
│              Qd  =  │                         │                           │
│                     │ Qθb I          Qbb I    │                           │
│                     └                         ┘                           │
│                                                                           │
│ Expanded form:                                                            │
│                                                                           │
│       ┌ Qθθ   0     0     Qθb   0     0   ┐                              │
│       │ 0     Qθθ   0     0     Qθb   0   │                              │
│       │ 0     0     Qθθ   0     0     Qθb │                              │
│ Qd =  │ Qθb   0     0     Qbb   0     0   │                              │
│       │ 0     Qθb   0     0     Qbb   0   │                              │
│       └ 0     0     Qθb   0     0     Qbb ┘                              │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 5. COVARIANCE PREDICTION                                                  │
│                                                                           │
│             P⁻ = Φ P⁺ Φᵀ + Qd                                             │
│                                                                           │
│ Block interpretation:                                                     │
│                                                                           │
│                      ┌                         ┐                           │
│                      │ Pθθ⁻             Pθb⁻  │                           │
│              P⁻  =   │                         │                           │
│                      │ Pbθ⁻             Pbb⁻  │                           │
│                      └                         ┘                           │
│                                                                           │
│ Pθθ⁻ = predicted attitude-error covariance                                │
│ Pbb⁻ = predicted gyro-bias covariance                                     │
│ Pθb⁻ = predicted attitude/bias cross covariance                           │
│ Pbθ⁻ = transpose(Pθb⁻)                                                     │
│                                                                           │
│ The -IΔt block in Φ causes gyro-bias uncertainty to propagate into        │
│ attitude uncertainty.                                                     │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 6. ACCELEROMETER MEASUREMENT MODEL                                        │
│                                                                           │
│ Normalize the measured acceleration:                                      │
│                                                                           │
│             z = ab / ||ab||                                               │
│                                                                           │
│ Predict gravity in the body frame:                                        │
│                                                                           │
│             ĝb = rotate_world_to_body(q⁻, ĝw)                             │
│                                                                           │
│ Measurement residual:                                                     │
│                                                                           │
│             r = z - ĝb                                                    │
│                                                                           │
│ Right-multiplicative measurement Jacobian:                                │
│                                                                           │
│                    ┌                         ┐                            │
│             H  =   │ [ĝb×]          0₃×₃   │                            │
│                    └                         ┘                            │
│                                                                           │
│ Expanded H matrix:                                                        │
│                                                                           │
│        ┌  0      -ĝz      ĝy      0   0   0  ┐                            │
│ H  =   │  ĝz      0      -ĝx      0   0   0  │                            │
│        └ -ĝy      ĝx      0       0   0   0  ┘                            │
│                                                                           │
│ Accelerometer direction-noise matrix:                                     │
│                                                                           │
│             R = σ²a I₃                                                    │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 7. INNOVATION COVARIANCE AND GATING                                       │
│                                                                           │
│ Innovation covariance:                                                    │
│                                                                           │
│             S = H P⁻ Hᵀ + R                                               │
│                                                                           │
│ Normalized innovation squared:                                            │
│                                                                           │
│             NIS = rᵀ S⁻¹ r                                                │
│                                                                           │
│ Accept the accelerometer update only when:                                │
│                                                                           │
│             | ||ab|| - g | ≤ magnitude tolerance                         │
│                                                                           │
│                         and                                               │
│                                                                           │
│             NIS ≤ configured innovation gate                              │
│                                                                           │
│ Rejected measurements leave q, b̂g, and P unchanged.                      │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 8. KALMAN GAIN AND ERROR-STATE CORRECTION                                 │
│                                                                           │
│ Kalman gain:                                                              │
│                                                                           │
│             K = P⁻ Hᵀ S⁻¹                                                 │
│                                                                           │
│ Matrix dimensions:                                                        │
│                                                                           │
│             P⁻     = 6 × 6                                                │
│             Hᵀ     = 6 × 3                                                │
│             S⁻¹    = 3 × 3                                                │
│             K      = 6 × 3                                                │
│                                                                           │
│ Estimated error state:                                                    │
│                                                                           │
│                     ┌ δθ̂ ┐                                               │
│             δx̂ = Kr = │    │                                             │
│                     └ δb̂ ┘                                               │
│                                                                           │
│ Expanded correction vector:                                               │
│                                                                           │
│             δx̂ = [δθ̂x δθ̂y δθ̂z δb̂x δb̂y δb̂z]ᵀ                        │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 9. NOMINAL-STATE INJECTION                                                │
│                                                                           │
│ Convert the estimated small-angle attitude error to a quaternion:         │
│                                                                           │
│             δq̂ ≈ normalize([1, ½δθ̂x, ½δθ̂y, ½δθ̂z])                     │
│                                                                           │
│ Correct the nominal attitude:                                             │
│                                                                           │
│             q⁺ = normalize(q⁻ ⊗ δq̂)                                      │
│                                                                           │
│ Correct the nominal gyro-bias estimate:                                   │
│                                                                           │
│             b̂g⁺ = b̂g⁻ + δb̂                                             │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 10. JOSEPH COVARIANCE UPDATE                                              │
│                                                                           │
│             A = I₆ - KH                                                   │
│                                                                           │
│ Joseph-form update:                                                       │
│                                                                           │
│             PJ = A P⁻ Aᵀ + K R Kᵀ                                        │
│                                                                           │
│ Matrix dimensions:                                                        │
│                                                                           │
│             I₆, A, P⁻, PJ = 6 × 6                                        │
│             K             = 6 × 3                                        │
│             H             = 3 × 6                                        │
│             R             = 3 × 3                                        │
│                                                                           │
│ The Joseph form helps preserve covariance symmetry and positive           │
│ semidefiniteness under floating-point arithmetic.                         │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 11. MEKF RESET                                                            │
│                                                                           │
│ After injecting δθ̂ into the nominal quaternion, reset the local attitude │
│ error coordinates using the reset Jacobian:                               │
│                                                                           │
│                        ┌                         ┐                         │
│                        │ I - ½[δθ̂×]       0    │                         │
│             Greset  =  │                         │                         │
│                        │      0             I    │                         │
│                        └                         ┘                         │
│                                                                           │
│ Reset the covariance:                                                     │
│                                                                           │
│             P⁺ = Greset PJ Gresetᵀ                                        │
│                                                                           │
│ Restore numerical symmetry:                                               │
│                                                                           │
│             P⁺ = ½(P⁺ + P⁺ᵀ)                                             │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 12. CORRECTED MEKF STATE                                                  │
│                                                                           │
│ Nominal state:                                                            │
│                                                                           │
│             q⁺                                                            │
│             b̂g⁺                                                          │
│                                                                           │
│ Corrected covariance:                                                     │
│                                                                           │
│                      ┌                         ┐                           │
│                      │ Pθθ⁺             Pθb⁺  │                           │
│              P⁺  =   │                         │                           │
│                      │ Pbθ⁺             Pbb⁺  │                           │
│                      └                         ┘                           │
│                                                                           │
│ The corrected state and covariance become the inputs to the next gyro     │
│ prediction cycle.                                                         │
└───────────────────────────────────┬───────────────────────────────────────┘
                                    │
                                    └──────────────► next timestep
### MEKF Prediction

Implemented prediction capabilities:

* Bias-corrected gyroscope measurement
* Rotation-vector integration
* Small-angle handling
* Incremental quaternion construction
* Right-multiplicative attitude propagation
* Quaternion normalization
* Six-state transition matrix
* Full covariance propagation
* Gyroscope white-noise contribution
* Gyro-bias random-walk contribution
* Attitude-bias cross-covariance propagation
* Covariance symmetry restoration
* Input and state validation
* Transactional state commit

Bias-corrected angular rate:

```text
omega_corrected =
    omega_measured -
    bias_estimated
```

Attitude prediction:

```text
attitude_new =
    normalize(
        attitude_old *
        delta_attitude
    )
```

Covariance prediction:

```text
P_new =
    Phi *
    P_old *
    transpose(Phi) +
    Qd
```

### MEKF Accelerometer Update

The accelerometer update uses the measured gravity direction to correct tilt.

Implemented capabilities:

* Accelerometer configuration validation
* Gravity-magnitude gating
* Measurement normalization
* Predicted body-frame gravity direction
* Gravity-direction residual
* Right-multiplicative measurement Jacobian
* Innovation covariance
* Guarded `3 × 3` matrix inversion
* Normalized innovation squared gating
* Six-state Kalman gain
* Multiplicative quaternion correction
* Gyro-bias correction
* Joseph-form covariance update
* MEKF reset covariance transformation
* Covariance symmetry restoration
* Rejected-update state preservation

```text
accelerometer measurement
          │
          ▼
magnitude validation
          │
          ▼
normalized measured gravity
          │
          ▼
predicted body-frame gravity
          │
          ▼
innovation and NIS gate
          │
          ▼
Kalman correction
          │
          ├── attitude correction
          ├── gyro-bias correction
          └── covariance update
```

The accelerometer update constrains tilt relative to gravity but cannot independently observe rotation about the gravity vector.

Latest recorded MEKF test result:

```text
Passes: 722
Fails:  0
Result: PASS
```

## MEKF Configuration

The current configuration includes:

```c
typedef struct
{
    VECTOR_3F initial_attitude_std_rad;
    VECTOR_3F initial_gyro_bias_std_rad_s;

    float gyro_noise_density_rad_s_sqrt_hz;
    float gyro_bias_random_walk_rad_s2_sqrt_hz;

    float accelerometer_direction_std;
    float gravity_magnitude_m_s2;
    float accelerometer_magnitude_tolerance_m_s2;
    float accelerometer_innovation_gate;

    float maximum_delta_time_s;
} MEKF_CONFIG;
```

These values define:

* Initial attitude uncertainty
* Initial gyro-bias uncertainty
* Gyroscope process noise
* Gyro-bias random walk
* Accelerometer direction uncertainty
* Expected gravity magnitude
* Allowed acceleration-magnitude difference
* Innovation rejection threshold
* Maximum permitted prediction timestep

## Repository Layout

```text
guidance-navigation-control/
├── README.md
├── LICENSE
├── NOTICE
├── CHANGELOG.md
├── CMakeLists.txt
├── docs/
│   ├── GNC_MASTER_PLAN.md
│   ├── decisions/
│   └── experiments/
├── include/
│   └── gnc/
├── src/
│   ├── attitude/
│   ├── control/
│   ├── guidance/
│   ├── math/
│   └── navigation/
├── simulation/
│   └── python/
├── platforms/
│   ├── host/
│   └── sdr-rev2/
├── tests/
│   ├── unit/
│   ├── integration/
│   └── data/
└── upstream/
    └── sdr-rev2/
        ├── SOURCE_REVISIONS.txt
        ├── math_sdr/
        └── mekf/
```

Files under `upstream/sdr-rev2/` preserve the current SDR reference implementation and source revisions.

Portable implementations will be developed separately under `src/` and `include/`.

## Engineering Conventions

| Quantity              | Convention                                  |
| --------------------- | ------------------------------------------- |
| Quaternion order      | `[w, x, y, z]`                              |
| Stored attitude       | Body to world                               |
| Gyroscope input       | radians per second                          |
| Acceleration          | meters per second squared                   |
| Magnetic field        | microtesla                                  |
| Pressure              | pascals                                     |
| Position              | meters                                      |
| Velocity              | meters per second                           |
| Timestamp             | integer microseconds                        |
| Internal calculations | SI units                                    |
| MEKF attitude error   | Right-multiplicative local body-frame error |

## Testing Strategy

```text
Unit Tests
    │
    ▼
Integration Tests
    │
    ▼
Synthetic Trajectories
    │
    ▼
Recorded Data Replay
    │
    ▼
Hardware Bench Testing
    │
    ▼
Hardware-in-the-Loop
    │
    ▼
Flight Validation
```

Planned quantitative outputs include:

* Quaternion angular error
* Quaternion norm
* Gyro-bias estimation error
* Innovation
* Normalized innovation squared
* Covariance diagonal values
* `±3σ` uncertainty bounds
* Velocity drift
* Position drift
* Estimator execution time
* Sensor update timing

## Current Development Priorities

1. Preserve and review the completed SDR MEKF accelerometer milestone.
2. Port the tested math, Mahony, and MEKF modules into portable interfaces.
3. Add a desktop CMake build.
4. Add automated host tests.
5. Add repeated MEKF predict/update sequence tests.
6. Add recorded sensor-data playback.
7. Add plots for attitude, bias, innovation, and covariance.
8. Validate Mahony and MEKF behavior on Rev 2 hardware.
9. Verify BMM150 axis mapping and compensated output.
10. Complete magnetometer hard-iron and soft-iron calibration before adding magnetic-field fusion.

## Planned Future Work

### Attitude Estimation

* Repeated MEKF convergence testing
* Long-duration covariance validation
* Magnetometer measurement update
* Magnetic innovation gating
* Mahony versus MEKF comparison
* Hardware timing and memory measurements

### Navigation

* Improved velocity integration
* Position integration
* Barometric altitude
* Barometric vertical velocity
* GPS position and velocity correction
* Local navigation-frame definition
* Flight-phase estimation
* INS/GPS/barometer fusion

### Guidance

* Desired attitude generation
* Rocket trajectory references
* Roll-control references
* Apogee targeting
* Satellite pointing references

### Control

* Quaternion-error control
* Angular-rate control
* PID
* Gain scheduling
* Saturation and anti-windup
* Servo allocation
* Reaction-wheel allocation
* Magnetorquer control

### Simulation and Validation

* Synthetic IMU trajectories
* Sensor-noise models
* Gyro-bias models
* Accelerometer vibration models
* Monte Carlo testing
* Recorded flight-data replay
* Software-in-the-loop testing
* Hardware-in-the-loop testing
* Mechanical turntable testing
* Vibration testing
* Flight validation

## Documentation

The complete architecture, equations, roadmap, testing strategy, validation plan, development workflow, and engineering decisions are maintained in:

[`docs/GNC_MASTER_PLAN.md`](docs/GNC_MASTER_PLAN.md)

## Upstream Reference and Attribution

Sun Devil Rocketry Flight Computer Rev 2 is the first reference platform for this repository.

Relevant publicly shareable reference files are preserved under:

[`upstream/sdr-rev2/`](upstream/sdr-rev2/)

The exact source repository revisions are recorded in:

[`upstream/sdr-rev2/SOURCE_REVISIONS.txt`](upstream/sdr-rev2/SOURCE_REVISIONS.txt)

Original copyright notices and applicable license terms are preserved in the imported source files.

## Project Goals

The long-term goal is to build a reusable engineering platform combining:

* Mathematical foundations
* Portable C implementations
* Automated testing
* Simulation
* Recorded-data replay
* Hardware validation
* Flight validation
* Engineering documentation
* Rocket GNC
* Spacecraft attitude determination and control

The repository should remain organized around stable GNC interfaces rather than a single board, mission, or estimator.

## License

This repository is licensed under the BSD 3-Clause License.

See:

* [`LICENSE`](LICENSE)
* [`NOTICE`](NOTICE)
