# Wrist Subsystem

The [`Wrist`](../../src/main/java/io/github/frc461/rowdy25/subsystems/wrist/Wrist.java) is the upper rotational joint of the doubly-jointed scoring mechanism — it changes the angle of the gripper relative to the elevator carriage. Single TalonFX Kraken with Motion Magic Expo, just like `Elevator` and `Pivot`, but with one crucial difference: its **soft limits are functions of elevator and pivot position**, not constants.

This dynamic-limit behavior is what prevents the wrist from rotating into the elevator frame or the chassis when the other two joints leave it no clearance.

## States

`Wrist.State` enumerates the same per-pose presets as `Elevator` and `Pivot`, including the `_AT_BRANCH` / `_ONE_CORAL_FROM_BRANCH` pair for L2/L3/L4. The `MANUAL` constant's "target" is `WristConstants.LOWER_LIMIT.apply(0.0, 50.0)` — i.e., the lower limit evaluated at elevator=0, pivot=50 — a deliberately conservative value used only for telemetry when the operator takes over.

## Dynamic Target Clamping

The crux of the subsystem is `setTarget(pivotPosition, elevatorPosition)`:

```java
public void setTarget(double pivotPosition, double elevatorPosition) {
    target = MathUtil.clamp(
        getState() == State.MANUAL ? lastManualPosition : getState().position,
        WristConstants.LOWER_LIMIT.apply(elevatorPosition, pivotPosition),  // dynamic min
        WristConstants.UPPER_LIMIT.apply(elevatorPosition)                  // dynamic max
    );
}
```

`LOWER_LIMIT` is a `BiFunction<Double, Double, Double>` keyed on `(elevatorPosition, pivotPosition)`; `UPPER_LIMIT` is keyed on elevator alone. Both encode the mechanical clearance geometry:

- When the elevator is retracted, the wrist can rotate to a wide range — there's nothing nearby to collide with.
- When the elevator is partially extended *and* the pivot is angled outward, the wrist's lower bound rises to keep the gripper above the chassis frame.
- When the elevator is at L4 height, the upper bound drops so the wrist can't pass through the elevator carriage on its way up.

The default `WristCommand` calls `setTarget(pivot.getPosition(), elevator.getPosition())` *every tick*, so the clamp re-evaluates with the live joint positions. This is fundamentally why the wrist follows the elevator/pivot gracefully through state transitions: the target moves continuously rather than jumping to an illegal position the moment a state changes.

## `holdTarget(pivotPosition)`

Same Motion-Magic-with-feedforward pattern as `Elevator` / `Pivot`:

```java
public void holdTarget(double pivotPosition) {
    wrist.setControl(request.withPosition(target)
                            .withFeedForward(WristConstants.G.apply(getPosition(), pivotPosition)));
}
```

`G` here is keyed on `(wristPosition, pivotPosition)` because the wrist's own gravity load depends on its angle (cos(wristAngle)) *and* the pivot's angle (which determines how much of the wrist's gravity vector is along its rotation axis). The feedforward is empirically tuned via [`GravityGainsCalculator`](../util/OTHER.md), the same calculator the pivot uses.

## `move(axisValue, pivotPosition, elevatorPosition)`

Manual joystick mode uses the dynamic limits *as the soft-stop distances*:

```java
wrist.set(axisValue > 0
    ? axisValue * expOutput(UPPER_LIMIT.apply(elevatorPosition) - getPosition(), 1, 5, 10)
    : axisValue * expOutput(getPosition() - LOWER_LIMIT.apply(elevatorPosition, pivotPosition), 1, 5, 10));
```

Same logistic soft-clip pattern as `Elevator.move` and `Pivot.move`, but the *limits themselves* shift as the operator moves the other joints. The midpoint `h = 5` is larger here because wrist degrees are smaller-magnitude than elevator inches.

## Hardware Configuration

Single TalonFX with integrated rotor encoder (no CANcoder), configured for Motion Magic Expo. The `SENSOR_TO_DEGREE_RATIO` makes `getPosition()` return degrees. Standard PID + `kV/kA`, plus `kG` is set to 0 in the slot configs because gravity is fed at runtime via `withFeedForward(...)`.

## Predicates

`isAtState(state)`, `isAtTarget()`, `nearTarget()` — analogous to `Elevator` / `Pivot`. The wrist does not need a `goingThroughStow` predicate because the dynamic-limit clamp prevents collision automatically.

## `setCoralScoringObstructedState(boolean)`

Same in-place obstruction-toggle pattern as the other two joints — flips `_AT_BRANCH` to `_ONE_CORAL_FROM_BRANCH` for the L2/3/4 states.

## `periodic()`

Updates telemetry only. The closed-loop control runs from `WristCommand`.

## `WristTelemetry`

Publishes position, target, state name, `isAtTarget`, `nearTarget`, plus the current dynamic-limit window for diagnosing why the wrist stopped short of a commanded angle.

## Implementation note on tuning

The two limit functions (`UPPER_LIMIT`, `LOWER_LIMIT`) are by far the most expensive thing to tune on the wrist — they encode the actual collision geometry of the mechanism. They are typically polynomial interpolations of measured clearance data taken on the practice field. The PID/feedforward gains are tuned via SysID. Everything lives in `Constants.WristConstants`.

## See Also

- [`WristCommand`](../commands/SUBSYSTEM_COMMANDS.md) — Default command; calls `setTarget(pivot, elevator)` then `holdTarget(pivot)` every tick.
- [`GravityGainsCalculator`](../util/OTHER.md) — Source of the gravity feedforward function.
- [`Pivot`](PIVOT.md) / [`Elevator`](ELEVATOR.md) — The two joints whose positions parameterize the wrist's dynamic limits and gravity model.
- [`RobotStates`](../ROBOT_STATES.md) — `orderedTransition` always commands the wrist's state setter; the dynamic-limit clamp converts the commanded preset into a feasible target on the fly.
