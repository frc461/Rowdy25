# Robot Class

The [Robot](../../src/main/java/io/github/frc461/rowdy25/Robot.java) class extends [TimedRobot](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/TimedRobot.html) and serves as the main entry point for the robot's control loop.

## Initialization

The `Robot` constructor:

- Initializes robot constants based on robot identity via `RobotIdentity.initializeConstants()` (MAC-address detection)
- Forwards local TCP ports for vision systems (PhotonVision on `5800`, Limelight on `5801`)
- Instantiates the [RobotContainer](ROBOT_CONTAINER.md)

## Lifecycle Methods

All periodic methods run at the `TimedRobot` default period (every 20 ms).

- `robotPeriodic()` — Runs every loop regardless of mode; executes the `CommandScheduler` and calls `robotContainer.periodic()` for telemetry
- `autonomousInit()` — Runs once at the start of autonomous; retrieves the selected auto command from `RobotContainer` and schedules it
- `autonomousPeriodic()` — Empty; not required because command-based scheduling drives all autonomous behavior
- `teleopInit()` — Runs once at the start of teleop; cancels the autonomous command if still running
- `teleopPeriodic()` — Empty; teleop behavior is driven by default commands and button-bound triggers
- `disabledInit()`, `disabledPeriodic()`, `disabledExit()` — Disabled-mode hooks (currently empty)
- `testInit()` — Cancels all scheduled commands when entering test mode
- `simulationPeriodic()` — Hook for Java simulation (currently empty)

## Command Scheduler

The WPILib [CommandScheduler](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/CommandScheduler.html) is executed inside `robotPeriodic()` to schedule and run all registered commands. Commands are automatically added/removed based on button presses, state-machine triggers, and command completion conditions.

## See Also

- [RobotContainer](ROBOT_CONTAINER.md) — Initialization hub for subsystems, controls, and autonomous
- [RobotStates](ROBOT_STATES.md) — Superstructure state machine driven by the scheduler
