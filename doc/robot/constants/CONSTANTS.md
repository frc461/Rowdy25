# Constants Class

[Constants](../../src/main/java/io/github/frc461/rowdy25/constants/Constants.java) is the centralized configuration object for the entire codebase. Its fields are populated at startup by [`RobotIdentity.initializeConstants()`](ROBOT_IDENTITY.md) from one of the per-robot variant classes under `constants/variants/`.

## Nested Configuration Classes

- **`AutoConstants`** — Named-command marker strings (e.g., `INTAKE_MARKER`, `OUTTAKE_MARKER`), PathPlanner path constraints, default velocities/accelerations
- **`VisionConstants`** — Camera mount offsets and vision tuning; further subdivided into:
  - `LimelightConstants`
  - `PhotonConstants`
  - `QuestNavConstants`
- **`ElevatorConstants`** — Motor IDs, position presets, soft limits, PID/feedforward gains
- **`PivotConstants`** — Motor ID, CANcoder ID/offset, rotation limits, gravity tuning, ratchet servo channel
- **`WristConstants`** — Motor ID, position presets, control gains, load-dependent gravity gains
- **`IntakeConstants`** — Motor ID, roller speeds per state, sensor ports/thresholds
- **`SwerveConstants`** — Module IDs, wheel geometry, max velocity/acceleration, PathPlanner module config

Additional top-level entries include `ALLIANCE_SUPPLIER` and the coral-station / scoring lookups used by `RobotContainer`.

## Robot Variants

Concrete values live in subclasses of the variant classes:

- `DefaultConstants` — baseline values used for every variant and for the alpha bot
- `CompConstants` — competition robot overrides
- `SimConstants` — simulation overrides
- `TestConstants` — test-bench overrides

[`RobotIdentity`](ROBOT_IDENTITY.md) selects the variant at startup based on the RoboRIO's MAC address.

## Usage

Access values through the nested-class fields, e.g. `Constants.ElevatorConstants.STOW_POSITION` or `Constants.SwerveConstants.MAX_VELOCITY`. Code should *never* hard-code numeric tuning values inline — add them to a variant instead.

## Tuning

Edit the appropriate variant class (`constants/variants/*.java`) to tune for a specific robot. Use [SysID](../util/OTHER.md) to characterize motors before committing feedforward gains.
