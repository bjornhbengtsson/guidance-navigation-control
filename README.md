# Guidance, Navigation, and Control

A portable engineering workspace for developing, testing, documenting, and validating Guidance, Navigation, and Control algorithms for rockets, spacecraft, desktop simulation, and future custom flight computers.

The project currently builds on publicly shareable work developed for Sun Devil Rocketry, while keeping reusable algorithms separate from STM32 HAL, board-specific drivers, telemetry, and application logic.

## Current Status

```text
Quaternion and frame mathematics     ██████████  Complete
Gravity compensation                 ██████████  Complete
World-frame velocity foundation      ███████░░░  Implemented; validation ongoing

Mahony attitude filter               ██████████  Software complete
Mahony unit testing                  ██████████  20,141 passes
Mahony hardware validation           ██░░░░░░░░  In progress

MEKF initialization                  ██████████  Complete
MEKF attitude prediction             ██████████  Complete
MEKF covariance prediction           ██████████  Complete
MEKF accelerometer correction        ██████████  Complete
MEKF unit testing                    ██████████  722 passes
MEKF hardware integration            ░░░░░░░░░░  Not started

Magnetometer fusion                  ░░░░░░░░░░  Calibration required first
Navigation fusion                    █░░░░░░░░░  Early foundation
Guidance                             ░░░░░░░░░░  Planned
Control                              ░░░░░░░░░░  Planned
Architecture
Hardware Sensors
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

Simulation, recorded-data replay, and automated tests connect
directly to the same portable estimator and navigation interfaces.

This text-based architecture is intentionally used in the main README because it remains readable in GitHub desktop, web, and mobile clients. More detailed diagrams are maintained in the engineering documentation.

Implemented Foundations
Quaternion and Frame Mathematics
Quaternion multiplication, normalization, conjugation, addition, and scaling
Body-to-world vector rotation
World-to-body vector rotation
Quaternion ordering: [w, x, y, z]
Stored attitude convention: body-to-world
Gravity Compensation
Rotate world gravity into the body frame
Remove gravity from accelerometer measurements
Rotate linear acceleration into the world frame
Integrate world-frame acceleration into velocity
Mahony Attitude Filter
Gyroscope quaternion propagation
Proportional accelerometer correction
Integral gyro-bias correction
Anti-windup
Accelerometer validity and flight-state gating
Gyro-only fallback
Extensive host-side unit tests
Multiplicative Extended Kalman Filter

Current six-state error model:

δx =
[
    δθx
    δθy
    δθz
    δbx
    δby
    δbz
]

Implemented capabilities:

Nominal body-to-world quaternion
Three-axis gyro-bias estimate
6 × 6 error-state covariance
Bias-corrected gyro prediction
Incremental quaternion propagation
Covariance state-transition propagation
Gyroscope white-noise contribution
Gyro-bias random-walk contribution
Covariance symmetry restoration
Accelerometer gravity-direction correction
Acceleration-magnitude gating
Normalized innovation squared gating
Kalman gain and gyro-bias correction
Joseph-form covariance update
Multiplicative attitude-error injection and reset
Transactional failure handling

Accelerometer correction constrains roll and pitch relative to gravity. It does not independently observe yaw.

Repository Layout
guidance-navigation-control/
├── docs/
│   └── GNC_MASTER_PLAN.md
├── include/
│   └── gnc/
├── src/
│   ├── attitude/
│   ├── control/
│   ├── guidance/
│   ├── math/
│   └── navigation/
├── tests/
│   └── unit/
└── upstream/
    └── sdr-rev2/
        ├── SOURCE_REVISIONS.txt
        ├── math_sdr/
        └── mekf/

The files under upstream/sdr-rev2/ preserve the current SDR reference implementation and source revisions. Portable implementations will be developed separately under src/ and include/.

Engineering Conventions
Quantity	Convention
Quaternion order	[w, x, y, z]
Attitude mapping	Body to world
Gyroscope input	radians per second
Acceleration	meters per second squared
Magnetic field	microtesla
Timestamp	integer microseconds
Internal calculations	SI units
MEKF attitude error	Right-multiplicative local body-frame error
Current Development Priorities
Preserve and review the completed SDR MEKF accelerometer milestone.
Port the tested math, Mahony, and MEKF modules into portable interfaces.
Add a desktop build and automated host test workflow.
Add repeated MEKF predict/update sequence tests.
Add recorded sensor-data playback and plotting.
Validate Mahony and MEKF behavior on Rev 2 hardware.
Validate and calibrate the BMM150 magnetometer before adding magnetic-field fusion.
Documentation

The complete system architecture, equations, development roadmap, validation strategy, design conventions, and current evidence are maintained in:

docs/GNC_MASTER_PLAN.md

Upstream Reference and Attribution

Sun Devil Rocketry Flight Computer Rev 2 is the first reference platform for this repository.

Relevant publicly shareable source files are preserved under:

upstream/sdr-rev2/

The exact source repository revisions are recorded in:

upstream/sdr-rev2/SOURCE_REVISIONS.txt

Original copyright notices and license terms are preserved.

License

This repository is licensed under the BSD 3-Clause License. See LICENSE and NOTICE.
