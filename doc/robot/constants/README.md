# Constants Overview

This directory documents the project's configuration model. The actual numeric values and tuning parameters live in `Constants.java` and in the per-robot variant classes under `constants/variants/`.

Key points:

- The project supports multiple robot variants (alpha, comp, sim, test). At startup the code calls `RobotIdentity.initializeConstants()` which selects the appropriate variant by reading the machine's MAC address. See `RobotIdentity.java` for the mapping logic.
- Edit the variant source files in `src/main/java/io/github/frc461/rowdy25/constants/variants/` to change values for a specific robot (do not edit the generated/active runtime selection logic unless you know what you're doing).
- For the full layout and descriptions of constant groups (Elevator, Pivot, Wrist, Intake, Swerve, Auto, Vision) see `CONSTANTS.md` in this directory.

Documentation files in this folder:

- `CONSTANTS.md` — detailed per-group descriptions and tuning checklist
- `ROBOT_IDENTITY.md` — how variants are selected at startup (MAC-based)
- `ROBOT_POSES.md` — field landmark poses used by pathfinding
