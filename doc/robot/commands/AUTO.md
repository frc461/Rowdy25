# Autonomous Commands

Autonomous-only commands for automated game-piece handling and conditional path following during the 15-second auto period. Live under [`commands/auto/`](../../src/main/java/io/github/frc461/rowdy25/commands/auto/).

## `SearchForObjectCommand`

Drives the swerve toward a vision-detected coral or algae using a three-stage state machine:

```java
public enum CommandStage { TO_OBJECT, SEARCH, WAIT }
```

### Lifecycle

- **`initialize()`** — Queries `PhotonUtil.Color.getRobotToBestObject(objectClass)` immediately. If *no* object is visible, `end = true` and the command finishes instantly. If an object is visible, `targetPose = swerve.localizer.bestCoralPose` (the field-frame projected approach pose computed in [`Localizer.updateRobotUtilityPoses`](../subsystems/LOCALIZER.md)) and stage is `TO_OBJECT`.

- **`execute()`** — Branches on the stage:
  - `TO_OBJECT` — Each tick, re-query `getRobotToBestObject` and refresh `targetPose` so the chassis chases the most up-to-date detection.
  - `WAIT` — Each tick, check for a target reappearance: if one is visible, transition back to `TO_OBJECT`; otherwise pin `targetPose = currentPose` (don't move).
  - `SEARCH` — Don't update `targetPose`; just continue driving to the wherever-we-pointed-after-the-loss waypoint.

  In both `TO_OBJECT` and `SEARCH` the chassis is commanded with the same velocity blend as [`PathfindToPoseAvoidingReefCommand`](DRIVE.md#3-velocity-profile) — `max(logistic, min(linear, maxVelocity))` — plus a PID yaw lock onto the target rotation.

  Then the X/Y/yaw tolerances are checked. When all three are met:
  - From `TO_OBJECT`: transition to `SEARCH`. Compute a new `targetPose` by interpolating 20% from the current pose toward the nearest reef center, with rotation interpolated 20% toward the reef bearing. This makes the robot wander a little toward the reef while waiting for vision to lock on a new piece — the rationale is that most coral spawns are *between* the chassis and the reef, so drifting in that direction increases the chance of re-detection.
  - From `SEARCH`: transition to `WAIT`, `swerve.forceStop()`. The chassis halts until vision sees a piece again.

- **`isFinished()`** — `end == true`, set when `objectObtained.getAsBoolean()` (e.g., the intake confirmed it has the piece) or when initialization saw no targets.

- **`end(interrupted)`** — `forceStop()` and pin `consistentHeading` to the current heading so the next teleop tick doesn't fight a stale lock.

### Why a three-stage machine?

A single "drive to detection" loop fails when the camera momentarily loses the target (typical with a moving robot — the detected centroid jitters). The three-stage machine gives the robot a *plan B*: drive to the last known position (`SEARCH`), then stop and re-acquire (`WAIT`). The 20% lerp toward the reef in the `TO_OBJECT → SEARCH` transition is a heuristic that biases the search drift toward where the *next* coral is statistically likely to be.

### `Swerve.directMoveToObject(...)` Wrapping

`Swerve.directMoveToObject(objectObtained, objectClass)` returns `new SearchForObjectCommand(this, fieldCentric, objectObtained, objectClass, 2.5)` — i.e., capped at 2.5 m/s. This is the entry point used by `RobotStates.groundCoralState` / `groundAlgaeState` (where it races against `intake::hasCoral` / `intake::algaeStuck`) and by `Swerve.pathFindTo*GroundIntakeCoral(...)` (where it runs after the standoff pathfind).

## `FollowPathRequiringAlgaeCommand`

A specialization of PathPlanner's `FollowPathCommand` that aborts mid-path if the color camera no longer sees an algae at scheduled checkpoints. Its purpose is to skip an algae-scoring leg of the autonomous routine when the algae acquisition failed earlier.

### Construction

Inherits from `FollowPathCommand` with the same pose/speeds/output/controller arguments used by `Swerve.AutoBuilder.configure(...)` — i.e., it integrates with the same PathPlanner controller. If `setAssumedPosition` is true, the path's starting holonomic pose is force-loaded into `localizer.setPoses` so the path starts from a known point.

### Algae-marker queue

```java
List<OneShotTriggerEvent> allInstantEvents = new ArrayList<>();
```

In `initialize()`, after the trajectory is computed, the trajectory's event list is scanned for `OneShotTriggerEvent`s whose name equals `Constants.AutoConstants.ALGAE_CHECK_MARKER`. These are the named markers placed in the PathPlanner UI at the points along the path where "we must have algae by here." They are queued into `allInstantEvents` and consumed in order.

### `execute()` algae check

```java
if (!allInstantEvents.isEmpty()
        && allInstantEvents.get(0).getTimestampSeconds() <= currentTime) {
    allInstantEvents.remove(0);
    if (!hasAlgaeTargets.getAsBoolean()) {
        interrupted = true;
    }
}
```

Each tick, if the head of the queue's timestamp has been reached, pop it and check `PhotonUtil.Color.hasAlgaeTargets`. If the camera sees no algae, set `interrupted = true`. `isFinished()` returns true on either path-complete or `interrupted`, so the command ends with `interrupted = true` propagated through `finallyDo`.

### `finallyDo` override

The class overrides `finallyDo(BooleanConsumer end)` to *prepend* its own `interrupted` flag onto the consumer:

```java
return new WrapperCommand(this) {
    @Override
    public void end(boolean wasInterrupted) {
        end.accept(FollowPathRequiringAlgaeCommand.this.interrupted || wasInterrupted);
        super.end(wasInterrupted);
    }
};
```

This is so downstream `andThen(...)` / sequencing code receives the *true* "we bailed out because the algae was missing" signal, not just "the path got canceled by something external." It is the only way for the auto routine to distinguish a successful algae path from a skipped one.

## Integration

Both commands are intended to be composed into the larger `AutoEventLooper` sequence built by [AutoManager](../autos/AUTO_MANAGER.md) or registered as `NamedCommands` for hand-authored PathPlanner autos.

`SearchForObjectCommand` is *also* used by teleop ground-intake states in `RobotStates.groundCoralState` / `groundAlgaeState` via `Swerve.directMoveToObject(...)`.

## See Also

- [AutoManager](../autos/AUTO_MANAGER.md) — Composes these commands into the final routine.
- [Drive Commands](DRIVE.md) — `PathfindToPoseAvoidingReefCommand` is the planner the dynamic auto routine uses for every drive segment; `SearchForObjectCommand` extends the same velocity-profile and yaw-PID patterns documented there.
- [Routines](../autos/ROUTINES.md) — `AutoEventLooper` and `AutoTrigger` polling model.
- [`Localizer.bestCoralPose`](../subsystems/LOCALIZER.md) — Source of the projected approach pose consumed by `SearchForObjectCommand`.
