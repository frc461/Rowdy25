# Robot Class

The [Robot](../../src/main/java/io/github/frc461/rowdy25/Robot.java) class extends WPILib's [`TimedRobot`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/TimedRobot.html) and is the program's main entry point. It is intentionally thin — almost all behavior is delegated to the command-based scheduler and to [`RobotContainer`](ROBOT_CONTAINER.md).

## Construction

`Robot()` performs three actions, in this order:

1. **`RobotIdentity.initializeConstants()`** — Reads the host's MAC address and rebinds the static fields of `Constants` to one of the variant classes (`CompConstants`, `SimConstants`, `TestConstants`, `DefaultConstants`). This **must** happen before any other class loads constants for hardware configuration, which is why it lives at the very top of the constructor. See [`ROBOT_IDENTITY.md`](constants/ROBOT_IDENTITY.md) for the dispatch table.
2. **`PortForwarder.add(5800, "photonvision.local", 5800)` and `PortForwarder.add(5801, "limelight.local", 5801)`** — Forwards the coprocessor configuration HTTP ports through the RoboRIO so a tethered laptop can reach the PhotonVision and Limelight dashboards over the robot radio's USB ↔ Ethernet bridge without needing a second IP route.
3. **`robotContainer = new RobotContainer()`** — Instantiates the full subsystem / chooser / binding graph. After this line returns the robot is fully wired and the scheduler can run.

## Lifecycle

`TimedRobot` calls the named hooks on a fixed period (the default 20 ms loop is used). The implementations are minimal:

| Hook | Behavior |
| --- | --- |
| `robotPeriodic()` | `CommandScheduler.getInstance().run()` (executes one tick: poll buttons → execute scheduled commands → schedule default commands) followed by `robotContainer.periodic()` (telemetry publish). |
| `autonomousInit()` | Calls `robotContainer.getAutonomousCommand()` (the most recently compiled `AutoManager` chain), caches it in `autonomousCommand`, and `schedule()`s it. |
| `autonomousPeriodic()` | Empty. The autonomous routine is itself a single `Commands.run(this::poll)` returned by `AutoEventLooper.cmd()`, so no per-tick code is needed here. |
| `teleopInit()` | Cancels the cached `autonomousCommand` if it is still running. This is the only place autonomous → teleop interlocking is handled; the rest is just the scheduler swapping default commands back in. |
| `teleopPeriodic()` | Empty. Teleop is driven entirely by the default commands (`DriveCommand`, `ElevatorCommand`, `PivotCommand`, `WristCommand`, `IntakeCommand`) and the trigger bindings established in `RobotContainer.configureButtonBindings()`. |
| `testInit()` | `CommandScheduler.getInstance().cancelAll()` — wipes the schedule clean for manual tuning. |
| `disabled*`, `testPeriodic`, `simulationPeriodic` | Intentionally empty — no per-mode side effects needed beyond what `TimedRobot` already does. |

### Why everything is empty

This is a deliberate command-based design choice. The scheduler called from `robotPeriodic()` is what actually executes work; `*Periodic()` hooks would either duplicate that work or fight it. The benefit is that *every* tick — autonomous, teleop, or test — converges through the same `CommandScheduler.run()` call, so the same telemetry and the same safety checks fire uniformly across modes.

## Loop Timing

The default 20 ms period (50 Hz) is the standard FRC loop rate. Anything that needs higher-rate updates (the Phoenix6 swerve modules, vision pipelines, etc.) runs in its own thread or on the coprocessor; the `Robot` thread only orchestrates command-based scheduling and aggregated telemetry. The Phoenix6 status signals consumed by `Swerve` are configured with their own update frequencies in `Swerve.java` and are read non-blocking each tick.

## See Also

- [`RobotContainer`](ROBOT_CONTAINER.md) — Initialization hub for subsystems, controls, and autonomous.
- [`RobotStates`](ROBOT_STATES.md) — Superstructure state machine driven by the scheduler.
- [`RobotIdentity`](constants/ROBOT_IDENTITY.md) — MAC-based constant variant selection executed in the constructor.
