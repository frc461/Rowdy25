# Wrist Subsystem
The [Wrist](../../src/main/java/io/github/frc461/rowdy25/subsystems/wrist/Wrist.java) class controls upper fine rotation for intake/outtake angle adjustment via TalonFX with motion magic and gravity compensation.
## States
Defines 13+ preset positions for coral/algae intake angles, branch scoring angles, and climb positions.
## Control
- Motion Magic Expo for smooth motion
- Gravity compensation tuned per load state
- Integrated or CANcoder encoder feedback
- Position presets in WristConstants
## Tuning
Edit position constants and gravity gains for field testing. Use SysID to characterize before deploying.
## See Also
- [Pivot](PIVOT.md), [Elevator](ELEVATOR.md) - Related superstructure subsystems
- [RobotStates](../ROBOT_STATES.md) - Wrist state transitions
