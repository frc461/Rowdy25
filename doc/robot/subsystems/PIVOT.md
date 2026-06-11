# Pivot Subsystem

The [Pivot](../../src/main/java/io/github/frc461/rowdy25/subsystems/pivot/Pivot.java) class manages base rotation (pitch) of the superstructure via a TalonFX with a CANcoder absolute encoder, controlled by Motion Magic Expo.

## States

`Pivot.State` defines the per-pose presets needed for coral and algae intaking and scoring (e.g. `STOW`, `L2_CORAL_AT_BRANCH`, `L3_CORAL_NEAR_BRANCH`, `GROUND_ALGAE`, `HIGH_REEF_ALGAE`, `PROCESSOR`, `NET`, `PREPARE_CLIMB`, `CLIMB`). Gravity feedforward gains are recomputed per load state (empty / coral / algae) using [`GravityGainsCalculator`](../util/OTHER.md).

## Safety

A servo-hub ratchet mechanism mechanically holds the pivot's position when disabled or in coast mode. The `Pivot` exposes ratchet engage / disengage helpers and a "cage intake" mode used during teleop. `RobotStates.orderedTransition()` consults pivot position to gate superstructure motion through known danger zones.

## Control

- Motion Magic for velocity-limited rotation
- Load-dependent gravity feedforward
- CANcoder magnet offset calibration for absolute position tracking
- Soft limits for mechanical bounds

## Tuning

Constants live in `Constants.PivotConstants` (with per-robot overrides under `constants/variants/`). Gravity gains require field characterization with the actual loaded mechanism.

## See Also

- [PivotCommand](../commands/SUBSYSTEM_COMMANDS.md) — Default command
- [Wrist](WRIST.md), [Elevator](ELEVATOR.md) — Related superstructure subsystems
- [RobotStates](../ROBOT_STATES.md) — Superstructure-level transitions
