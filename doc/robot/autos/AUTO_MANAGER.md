# AutoManager Class

[AutoManager](../../src/main/java/io/github/frc461/rowdy25/autos/AutoManager.java) is the heart of Rowdy25's autonomous: it observes a small set of SmartDashboard choosers and, every time the operator changes one, *recompiles* the entire autonomous routine into a fresh [`AutoEventLooper`](ROUTINES.md). No hand-written routine class per match — the routine is a runtime composition of enumerated reef positions, a closest-station selector, and modular `AutoTrigger`s chained by polling.

## Choosers

| Chooser name | Type | Effect |
| --- | --- | --- |
| `"Start Position"` | `SendableChooser<StartPosition>` | One of 6 discrete blue-alliance starting poses; `CUSTOM` skips the `localizer.setPoses(...)` pre-step so vision/QuestNav alone provides the initial pose. |
| `"Scoring or Algae Locations"` | `MultipleChooser<String>` | Ordered list of cycle keys drawn from `ScoringLocation.name() + Level.level` (e.g., `"AL4"`, `"BL2"`) and `Side.name()` (algae keys like `"FRONT"`). |
| `"Coral Station Preference"` | `SendableChooser<String>` | Forces `"station-1"` (driver-left) or `"station-2"` (driver-right) instead of the dynamic best-station algorithm. |
| `"Push Alliance Partner First"` | `SendableChooser<Boolean>` | If true, the routine starts with a `Swerve.pushAlliancePartnerOut()` segment before any pathfinding. |
| `"Intake Type"` | `SendableChooser<Boolean>` | If true, intermediate cycles use `pathFindToLeftCoralStationGroundIntakeCoral` / `pathFindToRightCoralStationGroundIntakeCoral` (which stop short of the station to give object-detection cameras a wider FOV). |

Every chooser's `onChange(...)` callback rebuilds `currentCommand = generateAutoEventLooper(robotStates).cmd()` from scratch, but only if `startPosition` and a non-empty scoring list have both been selected. This means operator edits propagate immediately to `getFinalAutoCommand()`, without leaving any leftover trigger state in the prior looper.

## `generateAutoEventLooper(...)` — Algorithm Walk-through

The method is the entire dynamic-routine compiler. Below is the algorithm in the order it executes.

### Inputs

- `startPosition : StartPosition` — one of 6 enumerated reef-relative slots. Each maps to a hard-coded blue-alliance `Pose2d` via `StartPosition.getStartingPosition(...)`, then alliance-flipped by `getStartingPose(...)` using `FlippingUtil.flipFieldPose(...)` when the alliance is red.
- `scoringOrAlgaeLocations : List<String>` — operator-ordered cycle keys.
- `coralStationOverride : String?`, `push : boolean`, `groundIntake : boolean` — user preferences.

### Step 1 — First segment

The head of the list is parsed by `getScoringLocation(...)` and, if that fails, `getAlgaeLocation(...)`. These do a linear search over `FieldUtil.Reef.ScoringLocation × Level` and `FieldUtil.Reef.Side` respectively. Each returns an `Optional<Pair<...>>` / `Optional<Side>` — both branches are handled with `ifPresentOrElse(...)`:

- **First step is a coral score:** schedule (in order):
  1. `InstantCommand(() -> localizer.setPoses(getStartingPose(startPosition)))` — but only if `startPosition.index != 0` (i.e., not `CUSTOM`).
  2. `ConditionalCommand(setStowState, setL2L3L4StowState, level == L1)` — pick the appropriate "carry coral" superstructure preset based on whether the first scoring level is L1.
  3. `pushAlliancePartnerOut().onlyIf(() -> push)` — optional bumper-push.
  4. `swerve.pathFindToScoringLocation(robotStates, location, level)` — the actual `PathfindToPoseAvoidingReefCommand` to the first branch.
- **First step is an algae score:** schedule
  1. `setPoses(...)` (same gate).
  2. `setStowState`.
  3. Optional push.
  4. `swerve.pathFindToAlgaeOnReef(...)` then `swerve.pathFindToNet(..., false)`.

The composed command is wrapped in `autoEventLooper.addTrigger("<startIdx>,<firstKey>", supplier)` and the returned `AutoTrigger` is appended to a local `triggersToBind` list.

### Step 2 — Cycle pairs

The method then iterates pairs `(current, next)` by repeatedly removing the head of `currentScoringLocations`:

```java
while (!currentScoringLocations.isEmpty()) {
    String current = currentScoringLocations.remove(0);
    Pose2d currentPose = resolvePose(current);   // discrete branch or algae-side pose
    if (currentScoringLocations.isEmpty()) break;
    String next = currentScoringLocations.get(0);
    // build trigger keyed "current,next"
}
```

`resolvePose(...)` selects between two pre-enumerated tables:

- `RobotPoses.Reef.getRobotPoseAtBranch(currentRobotScoringSetting, scoringLocation)` for coral cycles. There are exactly **12 reef branches × {AT_BRANCH, JUST_BEFORE_BRANCH, …} settings** — every entry is precomputed at startup from reef geometry, so resolution is an O(1) lookup.
- `RobotPoses.Reef.getRobotPoseAtAlgaeReef(side)` for algae cycles — one entry per reef side (6 total).

This discreteness is what makes the routine "intelligent" without solving anything in real time: every plausible cycle endpoint is a known finite pose.

### Step 3 — Choosing a coral station (specialized greedy algorithm)

Between two coral cycles the robot must visit a coral station. `getMostEfficientCoralStation(current, next)` is a minimal two-element minimization:

$$
\text{station}^\star = \arg\min_{i \in \{1, 2\}} \big(\,d(\mathbf{c}, \mathbf{s}_i) + d(\mathbf{s}_i, \mathbf{n})\,\big),
$$

where $\mathbf{c}, \mathbf{n}$ are the current and next *branch* translations and $\mathbf{s}_i$ are the two coral-station AprilTag translations from `FieldUtil.CoralStation.getCoralStationTags()`. Because there are only two stations, no general TSP solver is needed; the minimum-sum heuristic is provably optimal for this two-stop subproblem.

If `coralStationOverride` is non-null, that value bypasses the algorithm entirely. Otherwise the algorithm runs once per cycle pair, producing a station choice that depends on *both* the current pose and the chosen next branch — so swapping the operator's `next` selection automatically swaps the station too.

The result threads through `getPathFindingCommandToCoralStation(...)` (or `getPathFindingCommandToGroundIntakeCoral(...)` when `groundIntake` is true), which returns the corresponding `Swerve.pathFindToLeftCoralStation(...)` / `pathFindToRightCoralStation(...)` command.

### Step 4 — Build the inter-cycle trigger

For each cycle pair the supplier is:

```java
Commands.waitSeconds(0.5)                                // settling time after scoring
        .andThen(groundIntake
            ? getPathFindingCommandToGroundIntakeCoral(...)
            : getPathFindingCommandToCoralStation(...))
        .andThen(new WaitUntilCommand(
            () -> robotStates.stowState.getAsBoolean()
               || robotStates.intake.coralEntered()))    // dynamic wait
        .andThen(robotStates.swerve.pathFindToScoringLocation(
            robotStates, next.scoringLocation, next.level));
```

For an algae `next`, the wait is `0.5 s` (high algae — falls off the reef faster) or `1.0 s` (low algae — needs more settling), then `pathFindToAlgaeOnReef(...)` then `pathFindToNet(..., false)`.

This is the **modular** part: each cycle pair becomes an `AutoTrigger` keyed `"current,next"` and appended to `triggersToBind`. Because the suppliers close over the resolved pose tables and the chooser preferences *at construction time*, every recompile produces a fresh, self-contained chain.

### Step 5 — Wire up the polling chain

After the loop:

```java
autoEventLooper.active().onTrue(triggersToBind.get(0).cmd());

while (!triggersToBind.isEmpty()) {
    AutoTrigger current = triggersToBind.remove(0);
    current.done().onTrue(triggersToBind.isEmpty()
            ? Commands.none()
            : triggersToBind.get(0).cmd());
}
```

The looper's `active()` `Trigger` arms the first segment; each subsequent `done()` trigger arms the next, with the tail's `done()` scheduling `Commands.none()` so the chain terminates cleanly. See [ROUTINES.md](ROUTINES.md) for *why* this polling-chain pattern is more flexible than a `SequentialCommandGroup`.

## Public API

- `AutoManager(RobotStates robotStates)` — Constructor; registers all SmartDashboard choosers and seeds `currentCommand = Commands.none()` until the first valid chooser combination is selected.
- `Command getFinalAutoCommand()` — Returns the most recently compiled command (called by `RobotContainer.getAutonomousCommand()` at `autonomousInit()`).

## SmartDashboard Integration

All choosers are populated during `robotInit()` and remain editable while disabled. Operator changes recompile the routine immediately. The autonomous command returned by `getFinalAutoCommand()` is the most recent compilation result; selecting a new starting position 100 ms before `autonomousInit()` will use that new position.

## See Also

- [Pathfinder](PATHFINDER.md) — A standalone PathPlanner-wrapping utility. **Not used by `AutoManager`** — the Swerve helpers (`pathFindToScoringLocation`, `pathFindToLeftCoralStation`, etc.) build [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) instances directly.
- [Routines](ROUTINES.md) — Polling model and `AutoTrigger` / `AutoEventLooper` semantics.
- [RobotContainer](../ROBOT_CONTAINER.md) — Where `AutoManager` is instantiated.
- [RobotPoses](../constants/ROBOT_POSES.md) — Source of the enumerated branch / algae / station pose tables.
- [FieldUtil](../util/OTHER.md) — Source of the `ScoringLocation`, `Level`, `Side`, and `CoralStation` enums.
