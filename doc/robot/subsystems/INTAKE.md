# Intake Subsystem

The [`Intake`](../../src/main/java/io/github/frc461/rowdy25/subsystems/intake/Intake.java) is the gripper roller plus its sensing stack: a single TalonFX driving the roller wheels, a Redux Canandcolor sensor for proximity + RGB color, a digital beam-break across the gripper throat, and a debounced current-spike trigger for stalled-piece detection. It is the project's primary source of game-piece-presence truth.

## States

`Intake.State` is the per-mode motor-speed table:

- **`IDLE`** — 0 (or a small "hold algae" current if `maintainAlgaeCurrentOverride` is set).
- **`INTAKE`** / **`INTAKE_SLOW`** — pulling the piece in. The slow variant exists for the "barely-touching-coral" handoff at the coral station.
- **`INTAKE_OUT`** — *slight reverse during intake* — used to nudge a coral out of the gripper backwards when `Localizer.trustCameras` is false.
- **`INTAKE_OVERRIDE`** — operator forces an intake even if the sensors say a piece is held.
- **`OUTTAKE`** / **`OUTTAKE_SLOW`** / **`OUTTAKE_L1`** — eject; speeds tuned for L2-L4, soft drops, and L1 trough placement respectively.
- **`HAS_ALGAE`** — actively holding algae with maintenance current.

## Sensor Stack

```java
private final TalonFX        intake;          // roller motor
private final Canandcolor    canandcolor;     // proximity + RGB over CAN
private final DigitalInput   beamBreak;       // throat beam-break

public Trigger hasAlgaeOrCoralStuck = new Trigger(() -> Math.abs(getCurrent()) > 40.0)
        .debounce(0.1, DebounceType.kRising);
```

The Canandcolor is configured for **5 ms proximity / 25 ms color** integration in the constructor — fast proximity for tight detection windows, slower color so the RGB averages are stable. The lamp LED is turned off to avoid washing out the sensor when the lights are on.

The `hasAlgaeOrCoralStuck` `Trigger` watches motor current with a 100 ms rising-edge debounce, so transient current spikes don't fire the predicate; only a sustained spike does.

### Detection predicates

| Predicate | Logic | Meaning |
| --- | --- | --- |
| `beamBreakBroken()` | `!beamBreak.get()` | Beam-break across the throat is broken. |
| `coralEntered()` | `getProximity() < proximityObjectDetectionThreshold` | Canandcolor proximity sees something close. |
| `barelyHasCoral()` | `beamBreakBroken() OR coralEntered()` | Either sensor saw a piece — "maybe got it." |
| `hasCoral()` | `beamBreakBroken() AND coralEntered()` | Both sensors agree — "confirmed." |
| `coralStuck()` | `hasAlgaeOrCoralStuck && stallIntent == CORAL_STUCK` | Motor's hauling but the piece won't go in/out — typical jam. |
| `algaeStuck()` | `hasAlgaeOrCoralStuck && stallIntent == HAS_ALGAE` | Motor stall confirms algae captured (algae doesn't break the beam-break). |
| `hasAlgae()` | `maintainAlgaeCurrentOverride OR (hasAlgaeOrCoralStuck && stallIntent == HAS_ALGAE)` | Either currently in maintain-algae mode, or actively detected algae. |

The split between `barelyHasCoral()` and `hasCoral()` is what powers the driver bumper bindings: the bumpers conditionally pathfind to a branch only when `barelyHasCoral()` (one sensor agrees), but the auto routine waits for the more conservative `coralEntered()` before transitioning out of the station state.

### `StallIntent` enum

```java
public enum StallIntent { CORAL_STUCK, HAS_ALGAE }
```

The same `hasAlgaeOrCoralStuck` trigger gets interpreted differently depending on what the operator most-recently intended:

- After `setAlgaeIntakeState()`: a stall means algae is captured → algae-stuck branch fires, `hasAlgae()` returns true.
- After `setCoralIntakeState()`: a stall means the coral jammed → `coralStuck()` fires.

This is necessary because the algae is too soft to break the beam-break or trip the proximity, so the *only* signal for an algae grab is the motor stall.

### `proximityObjectDetectionThreshold`

Public `DoubleConsumer setter` (`setProximityObjectDetectionThreshold`) exposed for tuning from SmartDashboard or external state machines. The default threshold lives in `Constants.IntakeConstants`.

## `setIdleState()` — algae-aware

```java
public void setIdleState() {
    if (hasAlgae()) {
        maintainAlgaeCurrentOverride = true;
        setState(State.HAS_ALGAE);
    } else {
        maintainAlgaeCurrentOverride = false;
        setState(State.IDLE);
    }
}
```

This is the only state setter with branching: returning to idle while holding an algae *latches* the maintain-current state so the algae doesn't slip out. Releasing it requires explicitly transitioning to `OUTTAKE` (which clears the override).

The `setAlgaeIntakeState()` / `setCoralIntakeState()` setters set `stallIntent` and clear `maintainAlgaeCurrentOverride`. `setOuttakeState()` clears the override too, plus resets `stallIntent` to `CORAL_STUCK` (a safe default).

## `periodic()`

```java
@Override
public void periodic() {
    intakeTelemetry.publishValues();
    // (State → speed dispatch)
    Lights.setLights(hasCoral() || hasAlgae());
}
```

Dispatches the current `State` to a motor-speed setter (`intake.set(SPEED)`), publishes telemetry, and ticks the LED strip. The Lights call is the only inter-subsystem coupling — it tells the driver-side LEDs to indicate "we have a piece."

## `setIntakeSpeed(double)` Backdoor

Public, sets the motor speed directly. Used by SysID and one-off diagnostic flows; not used in production state transitions.

## `IntakeTelemetry`

Publishes `getColorReading` (RGB triple), proximity, `hasCoral`, `beamBreakBroken`, current state, motor current, and the stall trigger value. The RGB is the most useful debug signal for "is the Canandcolor seeing what I think it's seeing."

## Implementation note on tuning

The proximity threshold, the 40 A current-stall threshold, the 100 ms debounce, the per-state motor speeds, and the maintain-algae current are all tuned for the specific gripper geometry. The split between `barelyHasCoral` and `hasCoral` is a deliberate sensor-fusion choice — different downstream consumers want different levels of confidence.

## See Also

- [`IntakeCommand`](../commands/SUBSYSTEM_COMMANDS.md) — Default command; runs the state-machine dispatch from the operator's triggers.
- [`RobotStates`](../ROBOT_STATES.md) — Every coral-related state body calls `intake.setCoralIntakeState()` / `setOuttakeState()` / etc.
- [`Lights`](LIGHTS.md) — Consumes `hasCoral() || hasAlgae()` to drive the visual indicator.
- [`Swerve`](DRIVETRAIN.md) — `pathFindToNearestAlgaeOnReef` races against `intake.algaeStuck` to know when the algae has been captured.
