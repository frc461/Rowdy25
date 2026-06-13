# Elevator Subsystem

The [`Elevator`](../../src/main/java/io/github/frc461/rowdy25/subsystems/elevator/Elevator.java) is a TalonFX Kraken–driven leader/follower vertical lift, controlled with Phoenix6's Motion Magic Expo voltage profile and gravity-compensated for the *pivot-dependent* effective weight of everything above the carriage.

## States

The `State` enum is a list of `(name → target_inches)` pairs. The interesting structure is the per-level "at branch" vs "one coral from branch" pair: L2/L3/L4 each have both forms, and the choice between them is made by `RobotPoses.Reef.RobotScoringSetting` — selected upstream by [`Localizer`](LOCALIZER.md) based on `trustCameras` and the operator-level overrides. The `getL2State / getL3State / getL4State(mode)` helpers and the `setL2CoralState(mode) / ...` setters perform the dispatch:

```java
public State getL4State(RobotScoringSetting mode) {
    return switch (mode) {
        case L1, L2, AT_BRANCH         -> State.L4_CORAL_AT_BRANCH;
        case ONE_CORAL_FROM_BRANCH     -> State.L4_CORAL_ONE_CORAL_FROM_BRANCH;
    };
}
```

The `AT_BRANCH` height is slightly lower than the `ONE_CORAL_FROM_BRANCH` height because at full extension the lift must compensate for the chassis being one coral-width closer to the reef. The setters always reduce to `setState(State.X)`; the only state-with-side-effect is `setManualState()`, which also caches `lastManualPosition = getPosition()` so the joystick command knows what to hold when the operator releases.

`setCoralScoringObstructedState(isObstructed)` is the in-place toggle used when the L2/3/4 routine discovers (mid-flight) that the slot is blocked — it bumps the current state from `*_AT_BRANCH` to `*_ONE_CORAL_FROM_BRANCH` without re-entering the trigger chain.

## Hardware Configuration

Two Krakens, leader + Phoenix6 follower (inverted). Configuration applied in the constructor:

- **`FeedbackConfigs.SensorToMechanismRatio = ROTOR_TO_INCH_RATIO`** — the rotor-to-output gear ratio scaled so `getPosition()` returns *inches directly* without the caller needing to unit-convert.
- **`MotorOutputConfigs`** — invert + brake/coast neutral mode from `Constants.ElevatorConstants`.
- **`CurrentLimitsConfigs.SupplyCurrentLimit = CURRENT_LIMIT`** — protects the gearbox and the elevator strap from overcurrent if the carriage binds.
- **`AudioConfigs.AllowMusicDurDisable = true`** — required for the disable-time `Song.playRandom(...)` orchestra (see [`DRIVETRAIN.md`](DRIVETRAIN.md)).
- **`Slot0Configs`** — `kV, kA, kP, kI, kD` PID-with-feedforward (no `kG` here; gravity is fed at runtime as a function of pivot angle).
- **`MotionMagicConfigs`** — cruise velocity = 0 means "Motion Magic Expo mode," which generates an exponential motion profile parameterized by `EXPO_V` and `EXPO_A` instead of trapezoidal velocity/acceleration limits. The Expo profile is smoother for jerk-sensitive linear extensions because acceleration tapers rather than steps.

The follower is constructed inside a `try-with-resources` block:

```java
try (TalonFX elevator2 = new TalonFX(FOLLOWER_ID)) {
    elevator2.setControl(new Follower(LEAD_ID, true));
}
```

This is a Phoenix6 idiom: the `Follower` control is *latched* on the device firmware-side, so after `setControl` returns the local Java handle is no longer needed and is released by the try-with-resources.

`lowerSwitch` is a `DigitalInput` on `LOWER_LIMIT_SWITCH_DIO_PORT`, used for absolute homing.

## Position Control

```java
public void holdTarget(double pivotPosition) {
    checkLimitSwitch();
    elevator.setControl(request.withPosition(getTarget())
                               .withFeedForward(ElevatorConstants.G.apply(pivotPosition)));
}
```

This is the **default-command body**: every tick the elevator commands its target position with a feedforward voltage computed by `ElevatorConstants.G.apply(pivotPosition)`. Because the lift attaches *above* the pivot, the effective vertical component of the carriage's load varies with the pivot angle — `G` is a `DoubleUnaryOperator` (lookup or polynomial; see [`Constants`](../constants/CONSTANTS.md)) capturing that relationship. The math is straightforward: with pivot near vertical, the carriage is fully loaded; with pivot tipped over, the load is partially borne by the pivot mount. The exact form is empirically calibrated using [`GravityGainsCalculator`](../util/OTHER.md).

`getTarget()` returns either the state's `position` or `lastManualPosition` (in `MANUAL` mode).

## Manual Move (`move(axisValue)`)

Operator joystick override uses the `EquationUtil.expOutput` logistic to soft-clip near the limits:

```java
elevator.set(axisValue > 0
    ? axisValue * expOutput(UPPER_LIMIT - getPosition(), 1, 0.5, 10)   // logistic taper near top
    : axisValue * expOutput(getPosition() - LOWER_LIMIT, 1, 0.5, 10)); // logistic taper near bottom
```

The logistic is `M / (1 + e^{-k(d-h)})` with M=1, midpoint h=0.5, steepness k=10. Near the limit (`d → 0`) the output collapses smoothly toward 0; far from the limit it saturates at the joystick value. This is the soft-stop replacement for hard soft-limits — it preserves operator feel near the centerline while preventing slamming into the mechanical end-stops.

## `checkLimitSwitch()` Homing

```java
if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= LOWER_LIMIT)) {
    elevator.setPosition(LOWER_LIMIT);
}
```

If the switch is asserted *or* the dead-reckoned position has drifted below the soft-limit (i.e., the switch failed but we know we're at zero), re-anchor the integrated position to `LOWER_LIMIT`. `lowerSwitchTriggered()` is gated on `Constants.IDENTITY != ROWDY` — the original "Rowdy" robot did not have a working switch, so its identity disables the homing path entirely. This is a worked example of the [`RobotIdentity`](../constants/ROBOT_IDENTITY.md) variant pattern controlling runtime behavior.

## Predicates

| Predicate | Logic |
| --- | --- |
| `isAtState(state)` | `\|state.position - getPosition()\| < AT_TARGET_TOLERANCE` |
| `isAtTarget()` | `error < AT_TARGET_TOLERANCE` (uses the cached `error` updated in `periodic()`) |
| `nearTarget()` | `error < SAFE_TOLERANCE` (looser — used by `orderedTransition` to start the next mechanism early) |
| `goingDown(state)` | `getPosition() >= state.position` — i.e., "moving to `state` will reduce my position." This is what `RobotStates.orderedTransition` uses to pick branch A vs branch B of the safety choreography. |

## Periodic

```java
@Override
public void periodic() {
    elevatorTelemetry.publishValues();
    error = Math.abs(getPosition() - getTarget());
}
```

Note that *position control* is **not** issued from `periodic()`. The `ElevatorCommand` default command calls `holdTarget(pivotPosition)` every tick — keeping it inside the command lets the manual-axis branch coexist with the closed-loop branch without reinventing the state machine. The subsystem's own `periodic()` only updates telemetry and the cached error.

## `ElevatorTelemetry`

Publishes position (inches and meters), target, state name, `isAtTarget`, `nearTarget` to NetworkTables and DogLog. Topic table is in the source.

## Implementation note on tuning

`P/I/D`, `kV/kA`, `EXPO_V/EXPO_A`, `CURRENT_LIMIT`, `LOWER_LIMIT/UPPER_LIMIT`, every `State` height, and the `G` gravity-feedforward map were tuned via SysID and on-field iteration. The code documents the *roles*; the [`Constants`](../constants/CONSTANTS.md) page documents the structure of the tuning data.

## See Also

- [`ElevatorCommand`](../commands/SUBSYSTEM_COMMANDS.md) — Default command; bridges operator manual axis to `move(...)` and the auto target to `holdTarget(pivotPosition)`.
- [`Pivot`](PIVOT.md) / [`Wrist`](WRIST.md) — Together with `Elevator`, the doubly-jointed scoring mechanism.
- [`RobotStates`](../ROBOT_STATES.md) — `orderedTransition` consults `goingDown`/`nearTarget` to safely sequence elevator motion against pivot/wrist motion.
- [`Localizer`](LOCALIZER.md) — Owns `currentRobotScoringSetting` used by `getL2State / setL2CoralState(mode) / ...`.
