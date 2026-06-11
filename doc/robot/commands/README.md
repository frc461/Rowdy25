# Commands Documentation

Commands represent discrete actions the robot can perform; they extend WPILib's [Command](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html) class. The [CommandScheduler](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/CommandScheduler.html) manages command execution: scheduling them on button presses, state-machine triggers, default-command activation, or directly during autonomous.

## Command Lifecycle

- **`initialize()`** — Called once when the command is scheduled; sets initial state
- **`execute()`** — Called every scheduler iteration (~50 Hz) while the command is scheduled
- **`isFinished()`** — Polled every iteration; the command ends when it returns `true`
- **`end(boolean interrupted)`** — Called once on completion or interruption

## Requirements & Scheduling

Commands declare their subsystem dependencies via `addRequirements()`. The scheduler automatically:

- Prevents conflicting commands on the same subsystem from running simultaneously
- Interrupts a running command when a newly scheduled command requires the same subsystem
- Runs each subsystem's default command whenever no other command is requiring it

## Command Composition

Commands compose via fluent decorators:

- **`.andThen(command2)`** — Run sequentially
- **`.alongWith(command2)`** — Run in parallel
- **`.until(condition)`** — Interrupt when the condition becomes true
- **`.onlyIf(condition)`** — Schedule only if the condition is true at start
- **`.unless(condition)`** — Skip the command when the condition is true
- **`new ConditionalCommand(ifTrue, ifFalse, condition)`** — Choose between two commands based on a condition

## Rowdy25 Command Categories

- [Autonomous Commands](AUTO.md) — `SearchForObjectCommand`, `FollowPathRequiringAlgaeCommand`
- [Drive Commands](DRIVE.md) — `DriveCommand`, `PathfindToPoseAvoidingReefCommand`, `DirectMoveToPoseCommand`
- [Subsystem Commands](SUBSYSTEM_COMMANDS.md) — `ElevatorCommand`, `PivotCommand`, `WristCommand`, `IntakeCommand`

## See Also

- [RobotContainer](../ROBOT_CONTAINER.md) — Where commands are instantiated and bound to triggers
- [Subsystems](../subsystems) — Lower-level APIs that these commands drive
