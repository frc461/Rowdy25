# Commands Documentation
Commands represent actions the robot can perform, extending WPILib's [Command](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html) class. The [CommandScheduler](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/CommandScheduler.html) manages command execution: scheduling them on button presses, state transitions, or directly during autonomous.
## Command Lifecycle
- **initialize()** - Called once when scheduled; set initial state
- **execute()** - Called repeatedly (~50 Hz) while scheduled
- **isFinished()** - Called repeatedly; command ends when returning true
- **end(boolean interrupted)** - Called once upon completion or interruption
## Requirements & Scheduling
Commands declare their subsystem dependencies via ddRequirements(). The scheduler automatically:
- Prevents conflicting commands on the same subsystem
- Interrupts lower-priority commands when higher-priority ones are scheduled
- Runs default commands when no other command requires a subsystem
## Command Composition
Commands combine via fluent methods:
- **.andThen(command2)** - Execute sequentially
- **.alongWith(command2)** - Execute in parallel
- **.until(condition)** - Interrupt on condition
- **.onlyIf(condition)** - Only schedule if condition is true
- **ConditionalCommand(ifTrue, ifFalse, condition)** - Choose based on condition
## Rowdy25 Command Categories
- [Autonomous Commands](AUTO.md) - SearchForObjectCommand, path-following
- [Drive Commands](DRIVE.md) - DriveCommand, pathfinding, target alignment
- [Subsystem Commands](SUBSYSTEM_COMMANDS.md) - ElevatorCommand, PivotCommand, WristCommand, IntakeCommand
## See Also
- [RobotContainer](../ROBOT_CONTAINER.md) - Where commands are created and bound
- [Subsystems](../subsystems) - Lower-level APIs that commands invoke
