# Elevator Subsystem

The [Elevator](../../src/main/java/io/github/frc461/rowdy25/subsystems/elevator/Elevator.java) class manages vertical extension via TalonFX Kraken motors in a leader-follower configuration, driven with Motion Magic Expo voltage control.

## States

The `Elevator.State` enum defines presets used by the superstructure, including: `STOW`, `L2_L3_L4_STOW`, `CORAL_STATION`, `GROUND_CORAL`, `GROUND_ALGAE`, `L1_CORAL`, `L2_CORAL`, `L3_CORAL`, `L4_CORAL` (with `_AT_BRANCH` / `_NEAR_BRANCH` variants where applicable), `LOW_REEF_ALGAE`, `HIGH_REEF_ALGAE`, `PROCESSOR`, `NET`, `PREPARE_CLIMB`, and `CLIMB`.

## Control

- Motion Magic Expo for smooth, jerk-limited position tracking
- Gravity feedforward (constant on a linear lift)
- Limit-switch homing for encoder zeroing
- Soft limits to prevent mechanical damage
- `goingDown()` / `goingThroughStow()` predicates used by `RobotStates.orderedTransition()` to sequence superstructure motion safely

## Tuning

PID and feedforward gains live in `Constants.ElevatorConstants` and the per-robot variant overrides. Use [SysID](../util/OTHER.md) to characterize motor behavior before field deployment.

## See Also

- [ElevatorCommand](../commands/SUBSYSTEM_COMMANDS.md) — Default command and manual-control wrapper
- [RobotStates](../ROBOT_STATES.md) — Superstructure-level state transitions that drive the elevator
