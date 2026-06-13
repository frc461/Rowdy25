# RobotStates Class

[RobotStates](../../src/main/java/io/github/frc461/rowdy25/RobotStates.java) is the **superstructure controller** for everything that isn't the swerve drivetrain. It owns one instance of each non-drivetrain subsystem ([`Elevator`](subsystems/ELEVATOR.md), [`Pivot`](subsystems/PIVOT.md), [`Wrist`](subsystems/WRIST.md), [`Intake`](subsystems/INTAKE.md), [`Lights`](subsystems/LIGHTS.md)) plus the [`Swerve`](subsystems/DRIVETRAIN.md), exposes them as `public final` fields for direct access from `RobotContainer`, `AutoManager`, and commands, and exposes a fixed-vocabulary state machine that coordinates them.

The drivetrain is intentionally *not* part of the state vocabulary — it has its own independent `DriveMode` enum because driving and mechanism positioning are orthogonal concerns. `RobotStates` does, however, dispatch swerve `setBranchHeadingMode`, `setReefTagHeadingMode`, etc. as part of the per-state setup so the chassis heading lock matches the mechanism orientation.

## The `State` Enum

22 enumerated states, grouped by intent:

| Group | States | Purpose |
| --- | --- | --- |
| **Stow** | `STOW`, `L2_L3_L4_STOW` | Resting positions. `L2_L3_L4_STOW` is a *carrying* position used when the intake holds a coral and the operator-selected `currentAutoLevel` is L2+ — the pivot/wrist pre-rotate toward the reef so subsequent scoring transitions are short. |
| **Coral scoring** | `GROUND_CORAL`, `L1_CORAL`, `L2_CORAL`, `L3_CORAL`, `L4_CORAL` | One per reef level (plus the trough-level intake from the floor). |
| **Algae handling** | `GROUND_ALGAE`, `LOW_REEF_ALGAE`, `HIGH_REEF_ALGAE`, `PROCESSOR`, `NET` | Acquisition + two scoring locations. |
| **Coral station intake** | `CORAL_STATION`, `CORAL_STATION_OBSTRUCTED` | Two variants: flush against the station vs. one-coral-width away (used when the slot is blocked by an opponent or driver-partner robot). |
| **Climb** | `PREPARE_CLIMB`, `CLIMB` | Two-stage climb sequencing. |
| **Output / interrupt** | `OUTTAKE`, `OUTTAKE_ALGAE`, `OUTTAKE_L1`, `INTAKE_OUT` | Ejection variants. `OUTTAKE_L1` is the slower-speed L1 trough release; `INTAKE_OUT` is "spit the coral *backwards* out the intake" — used as a fallback when `Localizer.trustCameras` is false and the coral was loaded but never confirmed at the branch. |
| **Manual** | `MANUAL` | Direct joystick override; the per-subsystem default commands take over. |

Every enum constant has a Javadoc on the source file describing the corresponding physical pose; consult that for the mechanical geometry.

## Triggers — the "state is X" predicates

Every state has a public `Trigger` field that wraps `() -> currentState == State.X`. There are also composite "at-position" `Trigger`s like `atL2CoralState`, `atCoralStationState`, etc., which AND the three per-subsystem `isAtState(...)` predicates from `Elevator`, `Pivot`, and `Wrist`. The distinction matters:

- `l2CoralState` — "operator *requested* L2_CORAL"; fires the transition command.
- `atL2CoralState` — "mechanisms have *finished* moving to the L2_CORAL pose"; gates downstream actions like outtake.

The `at*OneCoralFromBranch*` variants pair with L2/L3/L4 to distinguish *flush* vs *one coral width away* sub-poses (selected by `Localizer.currentRobotScoringSetting`).

There are also two convenience composites:

```java
public final Trigger atReefAlgaeState     = atLowReefAlgaeState.or(atHighReefAlgaeState);
public final Trigger atAutoScoreState     = atL1CoralState.or(atL2CoralState).or(atL3CoralState).or(atL4CoralState)
                                            .or(atL2CoralOneCoralFromBranchState)
                                            .or(atL3CoralOneCoralFromBranchState)
                                            .or(atL4CoralOneCoralFromBranchState);
```

## `currentAutoLevel` and the listening / `needsUpdate` pattern

```java
private FieldUtil.Reef.Level currentAutoLevel = FieldUtil.Reef.Level.L4;
private final Trigger isListening = l1CoralState.or(l2CoralState).or(l3CoralState).or(l4CoralState);
private boolean needsUpdate = false;
```

This is a small, tricky piece of state. The operator can press POV directions on the op controller to change `currentAutoLevel` *while the robot is already at a scoring state* — for example, switching from L3 to L4 mid-approach. `setCurrentAutoLevel(...)` doesn't directly mutate `currentState`; instead it sets `needsUpdate = true` *if and only if* the robot is currently in any of the L*_CORAL listening states. Then in `configureToggleStateTriggers()`:

```java
isListening.and(() -> needsUpdate).onTrue(
    new InstantCommand(this::toggleAutoLevelCoralState)
        .andThen(() -> needsUpdate = false)
);
```

The `Trigger` polls every scheduler tick, so as soon as both predicates hold simultaneously the routine retoggles into the new level's state and clears the flag. The indirection prevents the operator from spamming POV → scheduling a flurry of state transitions; only the most recent level "wins" because earlier requests are all consumed within one or two ticks.

`setCurrentAutoLevel(...)` also pushes the L1/L2 override into `Localizer` so the scoring-setting machinery (`AT_BRANCH` vs `ONE_CORAL_FROM_BRANCH` etc.) takes the level into account.

## `orderedTransition(...)` — the safety choreography

The core of `RobotStates` is `orderedTransition(setPivotState, pivotState, setElevatorState, elevatorState, setWristState, fromL2L3L4Stow)`. Every state-transition command goes through it. It builds a `ConditionalCommand` that branches on `elevator.goingDown(targetElevatorState)`:

### Branch A — elevator going *down* (or staying)

```text
1. wrist → STOW
2. wait until wrist near target
3. if (pivot > 90° && !trustCameras) → pivot → PERPENDICULAR, wait
4. if (pivot.goingThroughStow(target)) → pivot → STOW, elevator → STOW, wait
5. setPivotState
6. setElevatorState
7. wait until elevator near target
8. setWristState
```

The pre-stowing of the wrist is critical: the wrist hangs *outboard* of the elevator carriage, so retracting the elevator with the wrist deployed will collide it into the chassis frame. Steps 3 and 4 add a second safeguard for the pivot: if the pivot is currently above 90° (i.e., past vertical) and the camera-trust is off, route it through perpendicular before continuing, because the gravity-feedforward map for "above vertical" is empirical and not safe to combine with arbitrary elevator motion.

### Branch B — elevator going *up* (or laterally without descent)

```text
1. if (pivot > 90° && !trustCameras) → pivot → PERPENDICULAR, wait
2. wrist → STOW (skipped when fromL2L3L4Stow=true; the carry-stow already had the wrist clear)
3. setPivotState
4. wait until pivot near target
5. setElevatorState
6. wait until elevator near target
7. setWristState
```

The ordering inverts on the way up so the pivot rotates out before the elevator extends — the inverse safety concern from branch A.

`fromL2L3L4Stow` is a hint that the previous state was the L2/L3/L4 carry-stow, in which case the wrist is already at a safe angle and re-stowing it would just cost time. The L2/L3/L4 coral state triggers pass `true` here.

## `configureToggleStateTriggers()` — state → command bindings

Each state trigger's `onTrue(...)` body is a small program. The structure is consistent across all of them:

1. **Set the drivetrain heading mode** (`swerve.setBranchHeadingMode`, `setCoralStationHeadingMode`, `setObjectHeadingMode`, etc.) — sometimes `.unless(DriverStation::isAutonomousEnabled)` so the auto routine retains its own heading control.
2. **Set the intake state** (idle / intake / outtake / algae-intake).
3. **`orderedTransition(...)`** to move the superstructure.
4. **A guard `WaitUntilCommand`** that watches for the natural exit condition (game-piece acquired, game-piece released, "we left the action zone") and stows.
5. **`.until(() -> !thisState.getAsBoolean())`** so the whole composition aborts cleanly the moment the operator toggles out.

A few patterns deserve highlight:

- **Auto-loop into carry-stow.** The `STOW` trigger's body includes `alongWith(WaitUntilCommand(intake.barelyHasCoral() && currentAutoLevel != L1).andThen(setL2L3L4StowState))` — i.e., the *moment* the intake detects a coral while currentAutoLevel is L2+, the robot auto-promotes to the carry-stow state. The inverse happens in the `L2_L3_L4_STOW` trigger: if the intake loses the coral *or* the operator switches down to L1, the robot demotes back to `STOW`. This is the entire reason `L2_L3_L4_STOW` exists as a distinct state — it lets the operator forget about stow management once the auto-level is set.
- **Object-tracking ground intakes.** `groundCoralState` and `groundAlgaeState` use `swerve.setObjectHeadingMode` and, after the superstructure is in position, `raceWith(WaitUntilCommand(intake::hasCoral))` against `swerve.directMoveToObject(...).asProxy()` — i.e., the swerve drives toward the detected coral/algae until either the intake confirms acquisition (race winner) or the operator releases. The `.asProxy()` is required because `directMoveToObject` requires the Swerve subsystem and the surrounding `RobotStates` composition would otherwise hoist that requirement onto itself.
- **Coral station obstruction fallback.** While in `CORAL_STATION`, an `alongWith(WaitUntilCommand(...isAgainstCoralStation && isStuck).andThen(toggleCoralStationObstructedState))` runs in parallel — if the robot is *trying* to dock with the station but the swerve reports it as stuck (couldn't reach), the routine auto-escalates to `CORAL_STATION_OBSTRUCTED`. The obstructed routine then has its own escape via `WaitUntilCommand(swerve.localizer::isAgainstCoralStation)`, which fires the moment the obstruction clears and reverts to `CORAL_STATION`.
- **Climb finalization.** `climbState` has `.andThen(pivot::setNormalMotionMagicProfile)` after `.until(...)` — the climb state uses a slower Motion Magic profile (because the pivot is bearing the full robot weight), and on exit the profile is restored to the default so subsequent moves don't crawl.

## `setDefaultCommands(driverXbox, opXbox)`

Installs:

- `Swerve` → `DriveCommand` (driver sticks).
- `Elevator` / `Pivot` / `Wrist` → respective subsystem commands, each parameterized with the operator's manual override axes (right stick Y, left stick Y, triggers depending on the mechanism).
- `Intake` → `IntakeCommand`, which is itself a state-machine over the intake's local `State` enum.

Subsystem-command implementations: see [`SUBSYSTEM_COMMANDS.md`](commands/SUBSYSTEM_COMMANDS.md).

## Pose-vs-state predicates

Three forwarders to the `Localizer` answer "is the chassis where this state expects it to be":

- `atScoringLocation()` — within `TRANSLATION_TOLERANCE_TO_ACCEPT` of the scoring pose for the *current* state.
- `nearStateLocation(state)` — within `TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE` of the scoring pose for a *named* state.
- `atTransitionStateLocation(state, autoTransition)` — within `TRANSLATION_TOLERANCE_TO_TRANSITION` (teleop) or `TRANSLATION_TOLERANCE_TO_TRANSITION_AUTO` (auto), used as the pre-arm condition for transitions that should fire *before* the chassis is fully docked. The auto-mode tolerance is the looser of the two because the auto chain has its own timing and shouldn't wait for the same docking precision that teleop demands.

## Telemetry

`publishValues()` writes the current state name to NetworkTables (`RobotStates/Current Robot State`) and forwards to each subsystem's telemetry class. The detailed topics are owned by the per-subsystem `*Telemetry` classes; this method is just the dispatch.

## Implementation note on the state vocabulary

The 22-state enum is intentionally exhaustive rather than parameterized — the team chose explicit named states over a sparser "(target, mode, has_coral)" tuple because the explicit form makes the state-trigger bindings one-to-one with operator intent, and reading the auto routine afterward only requires knowing the enum name. The number of `at*` triggers is unavoidable consequence of that choice.

## See Also

- [`RobotContainer`](ROBOT_CONTAINER.md) — Calls `configureToggleStateTriggers()` and `setDefaultCommands(...)`; binds operator buttons to the toggle methods.
- [`Subsystems`](subsystems/README.md) — Individual subsystem `State` enums mirror this class's vocabulary; the per-state setpoints live there.
- [`Swerve`](subsystems/DRIVETRAIN.md) — Owns the `DriveMode` enum referenced by the per-state heading-mode setters.
- [`Localizer`](subsystems/LOCALIZER.md) — Source of the pose predicates and `currentRobotScoringSetting`.
