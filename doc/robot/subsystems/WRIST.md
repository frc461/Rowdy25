# Wrist Subsystem

The [Wrist](../../src/main/java/io/github/frc461/rowdy25/subsystems/wrist/Wrist.java) class controls the upper fine rotation of the gripper for intake/outtake angle adjustment. It uses a TalonFX with Motion Magic Expo and load-aware gravity compensation.

## States

`Wrist.State` defines preset positions for coral intake, coral branch scoring (L1–L4), algae handling (ground, reef low/high, processor, net), and the climb sequence. The default `WristCommand` recomputes the target each cycle from current pivot and elevator positions so the gripper stays at a safe angle through superstructure motion.

## Control

- Motion Magic Expo for smooth motion
- Gravity compensation tuned per load state (empty / coral / algae)
- Encoder feedback (integrated or CANcoder, depending on robot variant)
- Position presets in `Constants.WristConstants`

## Tuning

Edit position constants and gravity gains in the appropriate per-robot variant under `constants/variants/`. Use [SysID](../util/OTHER.md) to characterize before deploying.

## See Also

- [WristCommand](../commands/SUBSYSTEM_COMMANDS.md) — Default command
- [Pivot](PIVOT.md), [Elevator](ELEVATOR.md) — Related superstructure subsystems
- [RobotStates](../ROBOT_STATES.md) — Superstructure-level transitions
