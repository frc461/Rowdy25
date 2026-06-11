# Subsystems Documentation

Subsystems are classes extending [SubsystemBase](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) that encapsulate the control logic for one mechanical or electronic system on the robot. Each subsystem provides:

- **State Management** — Discrete states and position setters for coordinated control
- **Motor Control** — Interface with CAN motors, encoders, and sensors
- **Closed-Loop Control** — PID, motion profiles, and feedforward to achieve target positions
- **Safety** — Soft limits, encoder homing, and mechanical constraints
- **Telemetry** — Publishing state data for debugging and monitoring (paired `*Telemetry.java` class)

## Rowdy25 Subsystems

- [Drivetrain (Swerve)](DRIVETRAIN.md) — Omnidirectional movement with four swerve modules
- [Elevator](ELEVATOR.md) — Vertical extension mechanism (TalonFX Kraken) with gravity compensation
- [Pivot](PIVOT.md) — Base rotation (pitch) with servo-hub ratchet safety engagement
- [Wrist](WRIST.md) — Upper fine rotation for intake/outtake positioning
- [Intake](INTAKE.md) — Game piece roller with sensor-based detection
- [Localizer](LOCALIZER.md) — Fusion of odometry and vision for field-relative pose estimation
- [Lights](LIGHTS.md) — LED strip control for status indication

## Subsystem Coordination

Individual subsystems define their own state enums (e.g. `Pivot.State.L2_CORAL_AT_BRANCH`) that mirror the robot-wide states defined in [RobotStates](../ROBOT_STATES.md). The `RobotStates` superstructure orchestrates these via `orderedTransition()` to ensure safe, non-colliding movement across all subsystems.

## Default Commands

Several subsystems have default commands (see [Subsystem Commands](../commands/SUBSYSTEM_COMMANDS.md) and [Drive Commands](../commands/DRIVE.md)). Default commands run automatically whenever no other command requires that subsystem, keeping teleop control responsive.
