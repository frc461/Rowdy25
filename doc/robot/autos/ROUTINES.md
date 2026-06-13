# Autonomous Routines & Event Polling

The classes in [`autos/routines/`](../../src/main/java/io/github/frc461/rowdy25/autos/routines/) implement the event-driven scaffolding that [`AutoManager`](AUTO_MANAGER.md) uses to glue dynamically-generated path segments together with superstructure state changes. Unlike a hand-authored `SequentialCommandGroup`, this scaffolding lets the autonomous routine *react* mid-flight — choosing the next path, station, or scoring level based on runtime sensor data — rather than committing to a fixed sequence at schedule time.

## Components

- **`AutoTrigger`** — A single addressable autonomous "step." Wraps a `Supplier<Command>` (lazy: the command is built only on first call), plus three observable flags (`isActive`, `isFinished`, `interrupted`) that are flipped by `beforeStarting(...)` / `finallyDo(...)` decorators on the underlying command. Exposes `Trigger` views (`active()`, `inactive()`, `interrupt()`, `done()`, `done(cyclesToDelay)`) bound to the parent looper's `EventLoop`.
- **`AutoEventLooper`** — Owns one private `EventLoop` and a list of `AutoTrigger`s. Its `cmd()` returns a long-running `Commands.run(this::poll)` that, every scheduler tick, calls `EventLoop.poll()` to re-evaluate every `Trigger` bound to it. This polling is the mechanism that fires "next-segment" `onTrue(...)` actions.

## The Polling Model

WPILib's `Trigger`-on-`EventLoop` API is the foundation. A `Trigger` is just a `BooleanSupplier` bound to a loop; the loop remembers each trigger's last value, and on each `poll()` call computes:

- Rising edge → run all `.onTrue(cmd)` bindings.
- Falling edge → run all `.onFalse(cmd)` bindings.
- True → run all `.whileTrue(cmd)` bindings, etc.

`AutoEventLooper.cmd()` is the only command actually scheduled by `CommandScheduler` during autonomous:

```java
return Commands.run(this::poll)              // every tick, poll the EventLoop
        .finallyDo(this::reset)              // on cancel/end, reset all triggers
        .beforeStarting(this::reset)         // on schedule, reset state
        .until(() -> !DriverStation.isAutonomousEnabled() || finishCondition);
```

So the autonomous *period* is a single `run(this::poll)` command. Every other command — every path-follow, every state-change `InstantCommand`, every `WaitUntilCommand` — gets scheduled by trigger callbacks fired from inside `poll()`.

### Why polling enables dynamic behavior

Three properties drop out of this design:

1. **Lazy command construction.** `AutoTrigger.cmd()` calls `triggeredCommand.get()` only on first access, and the supplier is set at looper construction. This means each step's `Command` instance — including any `Pose2d` arguments captured by `pathFindTo*(...)` — is captured *at construction time*, but the *condition* under which the next step fires is checked dynamically.
2. **Conditional advancement.** Because each transition is a `Trigger.onTrue(...)`, the loop can refuse to advance until a runtime condition is met. The auto cycle uses this directly:
   ```java
   Commands.waitSeconds(0.5)
           .andThen(/* pathfind to station */)
           .andThen(new WaitUntilCommand(
               () -> robotStates.stowState.getAsBoolean() || robotStates.intake.coralEntered()))
           .andThen(/* pathfind to next branch */);
   ```
   The `WaitUntilCommand` here decouples the next pathfind from a fixed timer — if a coral is detected by the intake beam-break early, the next path starts immediately; if the intake misses, the routine waits without breaking the chain.
3. **Composable cancellation.** `done(int cyclesToDelay)` returns a `Trigger` that fires only `cyclesToDelay` polls after `isFinished` flips to true. This lets the routine insert deterministic settling time without inserting a `Commands.waitSeconds(...)` into the middle of a path. `anyDone(...)` and `anyActive(...)` compose multiple triggers with `Trigger.or(...)` so a single transition can be gated on any of several upstream steps.

### Comparison: `SequentialCommandGroup` vs. `AutoEventLooper`

| | `SequentialCommandGroup` | `AutoEventLooper` |
| --- | --- | --- |
| When are sub-commands chosen? | At construction time, before scheduling. | At construction time, but transitions fire on observed `Trigger` rising edges every poll. |
| Can a step branch? | Only via `ConditionalCommand`. | Yes — any `Trigger` (sensor, pose, time-based) can `onTrue(stepN)` independently. |
| Can a step be skipped mid-routine? | No (it's a linear queue). | Yes — change the supplier, or wire multiple `done()` callbacks to alternate next-steps. |
| State observable externally? | Only via subclassing. | Yes — `isActive`, `isFinished`, `interrupted`, plus `done(cyclesToDelay)` views. |
| Useful for | Fixed, hand-tuned 15-second auto. | Operator-selected, dynamically composed auto with sensor-gated transitions. |

## How `AutoManager` Uses the Looper

`AutoManager.generateAutoEventLooper(...)` constructs the looper as follows (see [AUTO_MANAGER.md](AUTO_MANAGER.md) for the full algorithm):

1. Build one `AutoTrigger` per scoring/algae segment by `looper.addTrigger(name, supplier)`. The supplier captures the captured `Pose2d`s and the chosen pathfind helper.
2. Arm the first trigger by `looper.active().onTrue(triggers[0].cmd())`. `active()` is a `Trigger` over `(isActive && DriverStation.isAutonomousEnabled())`, so the chain only fires when the looper itself is running.
3. Chain consecutive triggers by `triggers[i].done().onTrue(triggers[i+1].cmd())`. The last trigger's `done()` schedules `Commands.none()` (terminator).
4. Return the looper. `AutoManager.getFinalAutoCommand()` then returns `looper.cmd()`.

At `autonomousInit()`:

- The command starts.
- First poll: `active()` rising edge → schedule `triggers[0].cmd()`. That command's `beforeStarting(...)` sets `isActive = true, isFinished = false`.
- N polls later, `triggers[0].cmd()`'s underlying composition (drive to start pose, push, pathfind to first branch) terminates. Its `finallyDo(...)` sets `isActive = false, isFinished = true`.
- Next poll: `triggers[0].done()` (which observes `inactive() && isFinished`) rises → schedule `triggers[1].cmd()`. And so on.

If the operator interrupts (`CommandScheduler.cancelAll()`) or autonomous ends, the outer `Commands.run` is canceled, `looper.reset()` clears every trigger's state, and the next autonomous run starts fresh.

## Named Commands

Named commands are a *separate* mechanism for hand-authored PathPlanner autos. They are registered via `RobotContainer.configurePathPlannerNamedCommands()` and bound to PathPlanner waypoint event markers placed inside `.path` files:

- `Constants.AutoConstants.OUTTAKE_MARKER` → `robotStates::toggleAutoLevelCoralState`
- `Constants.AutoConstants.INTAKE_MARKER` → `robotStates::toggleCoralStationState`

These named commands fire from `EventTrigger`s embedded in PathPlanner paths — they do **not** participate in the `AutoEventLooper` polling chain. They exist so that if a future routine bypasses `AutoManager` and runs a hand-authored `.auto` file directly, the intake / outtake markers still toggle the right superstructure states.

## See Also

- [PathPlanner Documentation](https://pathplanner.dev/)
- [AutoManager](AUTO_MANAGER.md) — Builds the looper from chooser selections.
- [RobotContainer](../ROBOT_CONTAINER.md) — Where `NamedCommands` are registered.
- [Commands](../commands/README.md) — `CommandScheduler` and decorator semantics that underlie `AutoTrigger.cmd()`.
