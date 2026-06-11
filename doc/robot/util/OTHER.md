# Other Utilities
Miscellaneous helper classes for field math, control, and custom triggers.
## Field Math
- **FieldUtil** - Computes target poses (reef branches, coral station, algae, processor, net, barge); calculates offsets for safe approach
- **RotationUtil** - Angle normalization and conversion utilities
- **EquationUtil** - Polynomial and interpolation utilities for smooth control
## Control & Characterization
- **SysID** - Motor characterization routines for feedforward and feedback tuning
- **PhoenixProfiledPIDController** - Motion Magic wrapper with smooth velocity profiles
- **ProfiledExpEndController** - Exponential end-of-motion deceleration for smooth stops
- **GravityGainsCalculator** - Utility for computing gravity compensation gains
## Custom Triggers & Helpers
- **DoubleTrueTrigger** - Detects simultaneous button presses (useful for modifier keys)
- **MultipleChooser** - Enhanced SmartDashboard chooser supporting multiple independent selections
- **EstimatedRobotPose** - Wrapper for vision-based pose measurements
- **MacAddress** - Robot identification via MAC address (used by RobotIdentity)
- **Elastic** - Data structure utilities for state composition
## See Also
- [Swerve Drive](../subsystems/DRIVETRAIN.md) - Uses FieldUtil for target calculations
- [Autonomous](../autos) - Uses FieldUtil and pathfinding utilities
