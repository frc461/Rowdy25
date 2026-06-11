# Intake Subsystem

The [Intake](../../src/main/java/io/github/frc461/rowdy25/subsystems/intake/Intake.java) class manages the game-piece roller motor and game-piece detection (CANandcolor proximity / color sensor plus an additional distance sensor).

## States

`Intake.State` drives the roller's motor output and includes: `IDLE`, `INTAKE`, `INTAKE_SLOW`, `OUT`, `OVERRIDE`, `OUTTAKE`, `OUTTAKE_SLOW`, `OUTTAKE_L1`, and `HAS_ALGAE`. The default `IntakeCommand` transitions between these based on coral/algae detection.

## Control

- Roller motor with configurable intake / outtake / slow speeds per state
- CANandcolor sensor for coral vs algae differentiation and proximity-based "has piece" detection
- Distance sensor used to detect coral-station obstruction
- Helpers such as `barelyHasCoral()` are consumed by driver bindings (e.g., the bumpers conditionally pathfind to a branch only when coral is held)

## Safety

When a piece is detected the intake automatically transitions to a holding state to prevent jamming or dropping; the `OUT` / `OVERRIDE` states give the operator a manual escape if the sensor disagrees with reality.

## Tuning

Roller speeds, beam-break / sensor thresholds, and timeouts live in `Constants.IntakeConstants` and per-robot variant overrides.
