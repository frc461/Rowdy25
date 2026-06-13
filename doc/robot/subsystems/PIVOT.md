# Pivot Subsystem

The [`Pivot`](../../src/main/java/io/github/frc461/rowdy25/subsystems/pivot/Pivot.java) is the base-mounted rotational joint of the doubly-jointed scoring mechanism. It uses two TalonFX Krakens (leader/follower) reading position from a *remote* CANcoder, plus a third TalonFX for the cage-intake roller during climb, plus two REV ServoHub channels for the up/down ratchet pawls.

This subsystem has the most complex per-tick state of the three superstructure joints because its gravity feedforward depends on **both** the wrist angle *and* the elevator height, and because the climb sequence interacts with two mechanical ratchets and a switchable Motion Magic profile.

## States

`Pivot.State` mirrors the names in [`RobotStates`](../ROBOT_STATES.md) plus a `PERPENDICULAR` safety state (used by `orderedTransition` to route the pivot through 90° when crossing dangerous angles). L2/L3/L4 each have `_AT_BRANCH` and `_ONE_CORAL_FROM_BRANCH` forms, just like `Elevator.State`. `getL2State / getL3State / getL4State(mode)` and `setL2CoralState(mode) / ...` dispatch on the `RobotScoringSetting`.

The `setClimbState()` setter has a side effect: it calls `setSlowMotionMagicProfile()` *before* setting the state, so the climb motion uses the slower profile from the very first command tick. The reverse — re-applying the normal profile — is done in `RobotStates.climbState` after the trigger exits.

## `MotionMagicProfile` enum

```java
public enum MotionMagicProfile {
    NORMAL(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0)        // Expo mode
        .withMotionMagicExpo_kV(...).withMotionMagicExpo_kA(...)),
    SLOW(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(0)        // Expo mode, smaller V/A
        .withMotionMagicExpo_kV(...).withMotionMagicExpo_kA(...));
}
```

`setNormalMotionMagicProfile()` / `setSlowMotionMagicProfile()` re-apply the active profile by calling `pivot.getConfigurator().apply(currentMotionMagicProfile.config)`. The CTRE configurator is a heavyweight call (it ships a CAN packet), so profile switches are reserved for state transitions — not per-tick.

## `RatchetState` enum

Two REV ServoHub channels drive two ratchet pawls:

- **Down ratchet** (always `ON`) — mechanically prevents the pivot from falling forward under gravity if power is lost or the motor is in coast.
- **Up ratchet** (`ON` by default, `OFF` during climb) — prevents the pivot from rotating *backward* past the climb position. Disengaged via `activateUpLatch` only when entering `CLIMB`-related states, because the climb sequence actually needs the pivot to rotate freely upward.

Each `RatchetState` constant carries a `pulseWidth` constant. The pulse widths are calibrated per servo for the specific pawl geometry on the robot.

## Hardware Configuration

```java
CANcoder encoder = new CANcoder(ENCODER_ID);
encoder.getConfigurator().apply(new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
        .withSensorDirection(ENCODER_INVERT)
        .withAbsoluteSensorDiscontinuityPoint(...)
        .withMagnetOffset(MAGNET_OFFSET)));

pivot = new TalonFX(LEAD_ID);
pivot.getConfigurator().apply(new TalonFXConfiguration()
    .withFeedback(new FeedbackConfigs()
        .withRemoteCANcoder(encoder)
        .withSensorToMechanismRatio(SENSOR_TO_DEGREE_RATIO))
    ...
    .withMotionMagic(currentMotionMagicProfile.config));
```

The CANcoder is the **source of truth** for pivot angle — the Talon's internal rotor encoder is only used as a velocity source. `withRemoteCANcoder(encoder)` configures the Talon to fuse the CANcoder over CAN as its position sensor. `SENSOR_TO_DEGREE_RATIO` is calibrated so `getPosition()` returns degrees directly. The CANcoder's `MAGNET_OFFSET` is the per-robot constant that aligns "encoder zero" with "physical zero pivot angle"; this is one of the values that changes per variant in [`constants/variants/`](../constants/ROBOT_IDENTITY.md).

`AbsoluteSensorDiscontinuityPoint` chooses where the wraparound happens (e.g., 0.5 for ±180°). The follower is set up identically to `Elevator` (inverted Phoenix6 `Follower`).

A third TalonFX (`intake`) drives the cage-intake roller — a separate motor used during climb to ingest the cage hooks.

## Gravity Feedforward

```java
private final GravityGainsCalculator gravityGainsCalculator = new GravityGainsCalculator(
    PivotConstants.AXIS_POSITION,
    ...
);
```

The gravity gain on the pivot is *non-trivial* because the effective center of mass of everything above the pivot — elevator carriage, wrist, gripper, plus whatever game piece is held — moves as both the elevator extends and the wrist rotates. `GravityGainsCalculator` (see [`OTHER.md`](../util/OTHER.md)) takes the three joint positions and returns a feedforward voltage proportional to `cos(θ_effective) · total_torque(extension)`.

Per tick, `holdTarget(elevatorPosition, wristPosition)`:

```java
currentG = gravityGainsCalculator.calculateGFromPositions(getPosition(), wristPosition, elevatorPosition);
pivot.setControl(request.withPosition(getTarget()).withFeedForward(currentG));
```

This is called from the default `PivotCommand`; the subsystem's own `periodic()` does not run closed-loop control (same pattern as `Elevator`).

## Predicates

- `isAtState(state)`, `isAtTarget()`, `nearTarget()` — analogous to `Elevator`, with tolerances in degrees.
- `goingThroughStow(state)` — `(state.position - STOW.position) * (getPosition() - STOW.position) < 0`. This is a sign-flip test: the product is negative iff `state.position` and `getPosition()` are on *opposite sides* of `STOW.position`, meaning the pivot must physically pass through the stow angle to reach the target. This is what triggers the elevator-pre-stow branch of `orderedTransition`.

## `activateCageIntake()` / `stopCageIntake()` / `periodic()` interaction

```java
public void activateCageIntake() { cageIntakeOverride = true; }
public void stopCageIntake()    { cageIntakeOverride = false; }

@Override
public void periodic() {
    pivotTelemetry.publishValues();

    if (currentState == State.PREPARE_CLIMB || cageIntakeOverride) {
        intake.set(0.6);
    } else {
        intake.set(0);
    }
    // (plus ratchet servo updates, LED indicator)
}
```

The cage-intake motor runs at 60% throttle whenever the pivot is in `PREPARE_CLIMB` or the operator has explicitly held the driver's POV-Left binding (which calls `activateCageIntake` while held, `stopCageIntake` on release). The override is non-mutually-exclusive with the state — i.e., the operator can spin the cage intake outside of climb if they need to.

## `move(axisValue)`

Same logistic-tapered soft-stop pattern as `Elevator.move(...)`. Manual control during state transitions is rare but the default `PivotCommand` exposes it for diagnostic use.

## `PivotTelemetry`

Publishes position, target, current state, ratchet states, "at target / near target," and the calculated `currentG` for debugging the gravity model. The DogLog stream is the primary source of post-match analysis for pivot oscillations.

## Implementation note on tuning

Beyond PID/feedforward gains, the magnet-offset, ratchet pulse widths, and the gravity-calculator geometric constants are *per-robot* — they change every variant of the chassis. The values live in `Constants.PivotConstants` and per-variant overrides; the math is what's documented here.

## See Also

- [`PivotCommand`](../commands/SUBSYSTEM_COMMANDS.md) — Default command; passes elevator and wrist positions into `holdTarget(...)`.
- [`GravityGainsCalculator`](../util/OTHER.md) — Computes the `currentG` feedforward.
- [`Elevator`](ELEVATOR.md) / [`Wrist`](WRIST.md) — The other two superstructure joints; their positions are inputs to the pivot's gravity model.
- [`RobotStates`](../ROBOT_STATES.md) — `orderedTransition` consults `goingThroughStow(state)` to gate elevator pre-stowing.
