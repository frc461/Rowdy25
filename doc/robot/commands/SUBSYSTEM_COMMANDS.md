# Subsystem Commands

Subsystem-specific commands provide higher-level control for individual superstructure subsystems. Each is the default command for its subsystem and blends manual-axis input with the position targets dictated by [RobotStates](../ROBOT_STATES.md).

## Commands

- **`ElevatorCommand`** — Default command for [Elevator](../subsystems/ELEVATOR.md). When the operator's manual axis exceeds the deadband it drives the elevator directly (and sets the manual flag); otherwise it holds the closed-loop target derived from the current pivot position.
- **`PivotCommand`** — Default command for [Pivot](../subsystems/PIVOT.md). Mirrors the elevator pattern: manual axis drives the pivot, otherwise the target is computed from elevator and wrist positions.
- **`WristCommand`** — Default command for [Wrist](../subsystems/WRIST.md). Manual axis drives the wrist; otherwise the target is recomputed each cycle from pivot and elevator positions so the gripper stays at a sane angle through superstructure motion.
- **`IntakeCommand`** — Default command for [Intake](../subsystems/INTAKE.md). A state-machine driver that switches between `INTAKE`, `INTAKE_SLOW`, `OUT`, `OVERRIDE`, `OUTTAKE` (with `_SLOW` / `_L1` variants), `HAS_ALGAE`, and `IDLE` based on beam-break and coral/algae detection.

## Implementation Pattern

Each command typically:

1. Declares its target subsystem as a requirement
2. Implements `initialize()`, `execute()`, `isFinished()`, `end()`
3. Reads operator joystick input (for manual control) or applies the state-driven target (for automated control)
4. Updates motor outputs via subsystem setters

These commands rarely "finish" on their own; they are interrupted when a state transition schedules a more specific command on the same subsystem.

## See Also

- [Subsystems](../subsystems) — Lower-level position setters these commands invoke
- [RobotStates](../ROBOT_STATES.md) — Owns the state targets that drive automatic mode
