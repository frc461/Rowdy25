# Subsystems Documentation
Subsystems are classes extending [SubsystemBase](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) that encapsulate the control logic for mechanical and electronic systems on the robot. Each subsystem provides:
- **State Management** - Discrete states and position setters for coordinated control
- **Motor Control** - Interface with CAN motors, encoders, and sensors
- **Closed-Loop Control** - PID, motion profiles, and feedforward to achieve target positions
- **Safety** - Soft limits, encoder homing, and mechanical constraints
- **Telemetry** - Publishing state data for debugging and monitoring
## Rowdy25 Subsystems
- [Drivetrain (Swerve)](DRIVETRAIN.md) - Omnidirectional movement with four swerve modules
- [Elevator](ELEVATOR.md) - Vertical extension mechanism via TalonFX with gravity compensation
- [Pivot](PIVOT.md) - Base rotation (pitch) with safety ratchet engagement
- [Wrist](WRIST.md) - Upper fine rotation for intake/outtake positioning
- [Intake](INTAKE.md) - Game piece roller and sensor-based detection
- [Localizer](LOCALIZER.md) - Fusion of odometry and vision for field-relative pose estimation
- [Lights](LIGHTS.md) - LED strip control for status indication
## Subsystem Coordination
Individual subsystems have their own state enums that mirror robot-wide states defined in [RobotStates](../ROBOT_STATES.md). The [RobotStates](../ROBOT_STATES.md) superstructure orchestrates these via orderedTransition() to ensure safe, coordinated movement across all subsystems.
## Default Commands
Some subsystems have default commands (e.g., Swerve drives with joystick input). Default commands run automatically when no other command requires that subsystem, ensuring responsive teleop control.
