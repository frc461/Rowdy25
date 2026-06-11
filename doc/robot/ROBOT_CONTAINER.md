# RobotContainer Class
The [RobotContainer](../../src/main/java/io/github/frc461/rowdy25/RobotContainer.java) is the initialization hub where all subsystems, controllers, and command bindings are configured.
## Key Components
- **RobotStates** - Superstructure state machine integrating all non-drivetrain subsystems
- **AutoManager** - Autonomous routine selector with dynamic path generation
- **SysID** - Motor characterization and tuning utility
- **Controllers** - Xbox controller bindings for driver (Index 0) and operator (Index 1)
## Constructor Sequence
1. Configure state-based triggers for RobotStates
2. Set default commands for all subsystems
3. Register PathPlanner named commands for autonomous path events
4. Configure button bindings from controller inputs
5. Initialize DogLog telemetry
## Critical Methods
- [configurePathPlannerNamedCommands()](../../src/main/java/io/github/frc461/rowdy25/RobotContainer.java) - Registers commands invoked by path waypoint markers
- [configureButtonBindings()](../../src/main/java/io/github/frc461/rowdy25/RobotContainer.java) - Maps Xbox controller buttons to robot actions
- [getAutonomousCommand()](../../src/main/java/io/github/frc461/rowdy25/RobotContainer.java) - Returns the selected autonomous command from AutoManager
## See Also
- [RobotStates](ROBOT_STATES.md) and its state transition system
- [Commands](commands) for implementation details
