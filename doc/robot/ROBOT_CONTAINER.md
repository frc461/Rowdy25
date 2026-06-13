# RobotContainer Class

The [RobotContainer](../../src/main/java/io/github/frc461/rowdy25/RobotContainer.java) is the initialization hub where all subsystems, controllers, and command bindings are configured.

## Key Components

- **RobotStates** — Superstructure state machine integrating the Swerve drivetrain with every non-drivetrain subsystem
- **AutoManager** — Autonomous routine selector with dynamic command generation
- **SysID** — Motor characterization and tuning utility (bound to the operator controller)
- **Controllers** — `CommandXboxController` instances for the driver (port 0) and operator (port 1)

## Constructor Sequence

1. Configure state-based triggers via `robotStates.configureToggleStateTriggers()`
2. Install default commands for all subsystems via `robotStates.setDefaultCommands(driverXbox, opXbox)`
3. Register PathPlanner named commands for autonomous path event markers
4. Configure controller button bindings
5. Initialize DogLog telemetry and the PDH logger
6. Set the PathPlanner pathfinder to `LocalADStar` and warm up both pathfinding and follow-path commands

## Critical Methods

- `configurePathPlannerNamedCommands()` — Registers commands (e.g. `INTAKE_MARKER`, `OUTTAKE_MARKER`) invoked by PathPlanner path event markers
- `configureButtonBindings()` — Maps driver and operator Xbox buttons to state toggles, pathfinding commands, and manual overrides
- `periodic()` — Called from `Robot.robotPeriodic()`; forwards to `robotStates.publishValues()` for telemetry
- `getAutonomousCommand()` — Returns the dynamically-built autonomous command from `AutoManager`

## See Also

- [RobotStates](ROBOT_STATES.md) — State transition system and toggle helpers
- [Commands](commands/README.md) — Implementation details for the commands bound here
- [AutoManager](autos/AUTO_MANAGER.md) — Source of the returned autonomous command
