# Constants Overview

This directory documents the project's configuration model. The actual numeric values and tuning parameters live in [`Constants.java`](../../src/main/java/io/github/frc461/rowdy25/constants/Constants.java) and the per-robot variant classes under `constants/variants/`.

## Key Points

- The project supports multiple robot variants (alpha, comp, sim, test). At startup the code calls `RobotIdentity.initializeConstants()`, which selects the appropriate variant by reading the host's MAC address. See [`RobotIdentity`](ROBOT_IDENTITY.md) for the mapping logic.
- Edit the variant source files in `src/main/java/io/github/frc461/rowdy25/constants/variants/` to change values for a specific robot. Do not edit the runtime selection logic itself unless you know what you're doing.
- For the full layout and descriptions of constant groups (Elevator, Pivot, Wrist, Intake, Swerve, Auto, Vision) see [`CONSTANTS.md`](CONSTANTS.md).

## Documentation Files

- [`CONSTANTS.md`](CONSTANTS.md) — Per-group descriptions and tuning notes
- [`ROBOT_IDENTITY.md`](ROBOT_IDENTITY.md) — How variants are selected at startup (MAC-based)
- [`ROBOT_POSES.md`](ROBOT_POSES.md) — Field landmark / robot-target poses used by pathfinding
