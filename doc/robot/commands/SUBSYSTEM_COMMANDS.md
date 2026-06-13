# Subsystem Commands

Each of the four superstructure subsystems has a permanent default `Command` that blends manual joystick input with the closed-loop "hold target" call. The commands live directly under [`commands/`](../../src/main/java/io/github/frc461/rowdy25/commands/) and are installed by `RobotStates.setDefaultCommands(driverXbox, opXbox)`.

This page documents each one's `execute()` body — they have no `initialize()` / `end()` / `isFinished()` logic because they are pure default commands that run as long as their subsystem has no other requirer.

## `ElevatorCommand`

```java
public void execute() {
    double axisValue = MathUtil.applyDeadband(manualAxisValue.getAsDouble(), Constants.DEADBAND) * 0.25;
    if (axisValue != 0.0) {
        elevator.setManualState();
        robotStates.setManualState();
        elevator.move(axisValue);
    } else {
        elevator.holdTarget(pivotPosition.getAsDouble());
    }
}
```

- Reads the operator's manual axis (right stick Y from the op controller), deadbands it, and scales by **0.25** so manual elevator motion is intentionally sluggish. The full-rate motion comes from the state preset; the operator axis is for *trim*, not gross motion.
- Non-zero axis → switch both `Elevator.State` and `RobotStates.State` to `MANUAL`, then call `elevator.move(axisValue)` (the logistic-tapered soft-stop, see [`ELEVATOR.md`](../subsystems/ELEVATOR.md)).
- Zero axis → call `elevator.holdTarget(pivotPosition)`. This single line is what does the Motion-Magic-Expo position command *plus* the pivot-angle-dependent gravity feedforward, every tick.

The promotion to `RobotStates.MANUAL` is a one-way trip: once the operator touches the axis, the entire superstructure is in manual mode until an explicit state setter is invoked (e.g., POV → Y → `setStowState`). This is intentional — it prevents the auto-toggle plumbing from fighting the operator's joystick.

## `PivotCommand`

Same structure as `ElevatorCommand`, but `holdTarget(elevatorPosition, wristPosition)` takes **two** position arguments because the pivot's gravity feedforward depends on both. The manual axis (left stick Y from the op controller) is scaled to a smaller fraction than the elevator because pivot rotational motion is more momentum-sensitive — overshooting can swing the entire mechanism into the chassis.

When the axis is active, the default command sets `Pivot.State` and `RobotStates.State` to `MANUAL` and calls `pivot.move(axisValue)`.

## `WristCommand`

```java
public void execute() {
    wrist.setTarget(pivot.getPosition(), elevator.getPosition());
    if (manualAxis != 0) {
        wrist.setManualState();
        robotStates.setManualState();
        wrist.move(axisValue, pivot.getPosition(), elevator.getPosition());
    } else {
        wrist.holdTarget(pivot.getPosition());
    }
}
```

Note that **`setTarget(pivot, elevator)` is called every tick before either branch**. This is the dynamic-limit-clamp mechanism: as the pivot and elevator move, the wrist's commanded target is continuously re-evaluated against the current clearance envelope (see [`WRIST.md`](../subsystems/WRIST.md)). The result is the wrist tracks the other joints' motion gracefully without ever overshooting into a collision.

In manual mode, the `move(axisValue, pivot, elevator)` call uses the same dynamic limits as the soft-stop distances.

## `IntakeCommand`

The intake command is fundamentally different — it has no manual axis, only the state-machine dispatch:

```java
public void execute() {
    switch (intake.getState()) {
        case INTAKE:
            if      (hasCoral() || algaeStuck())               setIdleState();
            else if (coralEntered() && !beamBreakBroken())     setIntakeSlowState();
            else if (beamBreakBroken() && !coralEntered())     setOuttakeSlowState();
            else                                                setIntakeSpeed(0.45);
            break;
        case INTAKE_SLOW: /* same logic but speed 0.15 */
        case INTAKE_OUT:        setIntakeSpeed( 0.65); break;
        case INTAKE_OVERRIDE:   setIntakeSpeed( 0.35); break;
        case OUTTAKE:           setIntakeSpeed(-0.50); break;
        case OUTTAKE_SLOW:      /* mirror of INTAKE_SLOW with negative speed */
        case OUTTAKE_L1:        setIntakeSpeed(-0.40); break;
        case HAS_ALGAE:         setIntakeSpeed( 0.03); break;
        case IDLE:              setIntakeSpeed( 0.00); break;
    }
}
```

The interesting branches are `INTAKE` / `INTAKE_SLOW` / `OUTTAKE_SLOW`. These run a *sensor-driven sub-state-machine*:

- **`hasCoral() || algaeStuck()` → IDLE.** Both sensors agree, or algae has stalled the motor — transition to idle (with `maintainAlgaeCurrentOverride` latched if `hasAlgae()`).
- **`coralEntered() && !beamBreakBroken()` → INTAKE_SLOW (or speed 0.15).** Proximity sees a piece but the beam-break hasn't broken yet — the coral is *almost* fully ingested. Slow down so it doesn't shoot through the gripper.
- **`beamBreakBroken() && !coralEntered()` → OUTTAKE_SLOW (or reverse-0.15).** Beam-break is broken but proximity says nothing close — the coral has passed *through* the gripper. Reverse slowly to bring it back into the proximity zone.
- **Neither → full intake speed (0.45).** Nothing in the gripper yet, hunt for the piece.

This is essentially a one-tick lookahead servo: each tick the command re-classifies the gripper state from the two binary sensors and picks the speed that nudges the coral toward the "both sensors agree" attractor. The transition into `IDLE` is the natural absorbing state.

`HAS_ALGAE` runs a small forward current (0.03) to keep the algae compressed against the gripper without overheating the motor. The exact value was tuned for the gripper geometry; the comment in the source notes it as still under test.

## Why the Sub-State Machine Lives in the Command

Putting it in `IntakeCommand` rather than `Intake.periodic()` keeps the subsystem narrowly responsible for "set speed" and "expose sensors." The command's `execute()` knows about *intent* (intake vs. outtake) and the sub-state machine that fuses sensor readings into the appropriate speed; the subsystem knows about *hardware* (the TalonFX, the Canandcolor, the beam-break). This is the standard command-based separation — the subsystem is a value object, the command is the policy.

## See Also

- [`RobotStates.setDefaultCommands(...)`](../ROBOT_STATES.md) — Installs these four commands.
- [`Elevator`](../subsystems/ELEVATOR.md) / [`Pivot`](../subsystems/PIVOT.md) / [`Wrist`](../subsystems/WRIST.md) / [`Intake`](../subsystems/INTAKE.md) — Per-subsystem implementation details.
- [`Constants.DEADBAND`](../constants/CONSTANTS.md) — Joystick deadband used by all three position commands.
