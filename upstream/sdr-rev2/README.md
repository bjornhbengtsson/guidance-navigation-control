# Sun Devil Rocketry Rev 2 Reference

This directory records publicly shareable reference material and source
provenance from the Sun Devil Rocketry Flight Computer Firmware repositories.

The portable GNC implementation must not depend directly on this directory.
Reference files are retained to:

- trace algorithm origins;
- compare behavior;
- preserve source revisions;
- support future porting and regression testing.

The initial source revisions are recorded in `SOURCE_REVISIONS.txt`.

Relevant upstream areas include:

- `mod/math_sdr`
- `mod/mahony`
- `mod/mekf`
- `mod/sensor`
- `test/math_sdr`
- `test/mahony`
- selected driver interfaces

Board-specific application code, generated output, private data, credentials,
telemetry systems, ignition logic, and unrelated drivers should not be copied
into the portable GNC core.
