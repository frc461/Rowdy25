# Other Utilities

Miscellaneous helper classes covering field math, control, characterization, and custom triggers.

## Field Math

- **`FieldUtil`** — Computes target poses (reef branches, coral stations, algae, processor, net, barge) and "nearest landmark" queries; alliance-aware
- **`RotationUtil`** — Angle normalization and conversion helpers
- **`EquationUtil`** — Polynomial / interpolation utilities for smooth control

## Control & Characterization

- **`SysID`** — Wraps WPILib's `SysIdRoutine` for the Swerve drive subsystem; bound to operator-controller chord triggers in `RobotContainer`
- **`PhoenixProfiledPIDController`** — Motion-magic-flavored wrapper around `ProfiledPIDController` for Phoenix 6 mechanisms
- **`ProfiledExpEndController`** — Profiled PID controller with exponential end-of-motion deceleration for smooth stops
- **`GravityGainsCalculator`** — Utility for computing pivot/wrist gravity feedforward gains from mechanism geometry

## Triggers & Helpers

- **`DoubleTrueTrigger`** — Custom `Trigger` that fires only when two source triggers are simultaneously true (useful for chord bindings like `leftBumper + rightBumper`)
- **`MultipleChooser`** — Extended `SendableChooser` that supports multiple independent selections (used by `AutoManager`)
- **`EstimatedRobotPose`** — Wrapper around a vision-derived pose with timestamp and standard deviations
- **`MacAddress`** — Reads the host machine's MAC address (used by [`RobotIdentity`](../constants/ROBOT_IDENTITY.md))
- **`Elastic`** — Helpers for publishing Elastic dashboard notifications/widgets

## See Also

- [Swerve](../subsystems/DRIVETRAIN.md) — Uses `FieldUtil` for target calculations and `SysID` for characterization
- [Autonomous](../autos) — Uses `FieldUtil` and the pathfinding helpers
