# RobotContainer Class

The [RobotContainer](../../src/main/java/io/github/frc461/rowdy25/RobotContainer.java) is where the subsystem graph, controller bindings, autonomous chooser, telemetry, and PathPlanner warmup are wired together. Everything else in the project is either reachable from this class or instantiated by something it instantiates.

## Field-Order Construction

The four owned instances are constructed as field initializers so they execute *before* the constructor body. The order matters: `RobotStates` builds every subsystem; `AutoManager` consumes that `RobotStates`; `SysID` consumes `RobotStates.swerve`. If you reorder these, the dependent constructors NPE.

```java
private final RobotStates robotStates = new RobotStates();
private final AutoManager autoManager = new AutoManager(robotStates);
private final SysID     sysID         = new SysID(robotStates.swerve);

private final CommandXboxController driverXbox = new CommandXboxController(0);
private final CommandXboxController opXbox     = new CommandXboxController(1);
```

## Constructor Sequence

After the field initializers run, the body of `RobotContainer()` does:

1. **`robotStates.configureToggleStateTriggers()`** — Installs every `State`-to-command binding on the global command-scheduler event loop. See [`ROBOT_STATES.md`](ROBOT_STATES.md) for what the bindings *do*; semantically, this is what makes "toggling a state" actually move motors.
2. **`robotStates.setDefaultCommands(driverXbox, opXbox)`** — Attaches `DriveCommand`, `ElevatorCommand`, `PivotCommand`, `WristCommand`, and `IntakeCommand` to their owning subsystems. These run whenever no other command requires the subsystem.
3. **`configurePathPlannerNamedCommands()`** — Registers the two intake/outtake event-marker hooks (below).
4. **`configureButtonBindings()`** — Wires every driver and operator button. The full table is below.
5. **`DogLog.setOptions(...)` and `DogLog.setPdh(new PowerDistribution())`** — Configures the team's chosen logger. The options tuple is `(captureNt=()->false, captureDs=false, logExtras=true, logExceptions=true, ntPublish=false, queueCap=5000, captureConsole=()->false, fastPolling=true)` — chosen so log files contain custom topics + exceptions only (no NT mirror, no DS replay), and the PDH is polled at the fast cadence for current-draw diagnostics. The exact tuple was tuned for log size vs. signal coverage on the practice field.
6. **`Pathfinding.setPathfinder(new LocalADStar())`** — Selects PathPlanner's local A* implementation as the active pathfinder. Even though Rowdy25's auto and teleop drive use `PathfindToPoseAvoidingReefCommand` (not the PathPlanner pathfinder), the planner is still registered because `FollowPathCommand` and any hand-authored PathPlanner autos that *do* call `AutoBuilder.pathfindToPose` depend on it being installed before they are scheduled.
7. **`PathfindingCommand.warmupCommand().schedule()` and `FollowPathCommand.warmupCommand().schedule()`** — Warmup commands force PathPlanner's JIT-heavy graph code to compile / load classes during disabled init, so the first real pathfinding call in autonomous doesn't take an unpredictable 100+ ms hit. The warmup runs once and self-terminates.

## Named Commands

```java
NamedCommands.registerCommand(Constants.AutoConstants.OUTTAKE_MARKER, new InstantCommand(robotStates::toggleAutoLevelCoralState));
NamedCommands.registerCommand(Constants.AutoConstants.INTAKE_MARKER,  new InstantCommand(robotStates::toggleCoralStationState));
```

These keywords are matched against PathPlanner waypoint event markers in any `.path`/`.auto` file loaded from `src/main/deploy/pathplanner/paths/`. They are **not** invoked by the dynamic [`AutoManager`](autos/AUTO_MANAGER.md) routine, which has its own coral-station entry/exit logic (see [`AutoEventLooper`](autos/ROUTINES.md)). They exist so hand-authored backup autos can still drive the superstructure.

## Driver Bindings

The driver controller (port 0) is the primary "drive + score" surface. The trigger model is the standard WPILib `CommandXboxController`: `.a()`, `.b()`, etc. expose `Trigger`s that you chain with `.onTrue()` / `.whileTrue()` / `.onFalse()`.

| Button | Bind | Effect |
| --- | --- | --- |
| **A** | `onTrue(toggleAutoHeading → toggleTrustCameras → rumble(0.25 s) if autoHeading)` | Composite chord: flips the swerve auto-heading mode *and* the camera-trust flag in lockstep, then pulses both rumble motors for 250 ms iff auto-heading came on. The two flags are tied because "use vision target lock" and "trust vision pose" are operationally the same decision. |
| **B** | `whileTrue(pathFindToProcessor)` | Holds the processor pathfind active for as long as B is held; releasing cancels and the swerve falls back to teleop drive. |
| **X** | `whileTrue(pathFindToNet(randomize=true))` | Holds the net pathfind; the second argument true randomizes the X target on the net to make the score harder to defend. |
| **Y** | `onTrue(setStowState)` | Force the superstructure into `STOW`. |
| **POV ↑** | `onTrue(setRotations(alliance == Red ? π : 0))` | Driver-relative gyro reset (away from driver station). |
| **POV ↓** | `onTrue(syncRotations)` | Snaps the pose estimator's heading to the gyro's current raw heading. |
| **POV ←** | `whileTrue(activateCageIntake/stopCageIntake)` | Runs the cage-intake wheels on the pivot mechanism while held. |
| **POV →** | `onTrue(escalateClimb)` | Advances the climb state machine one step (`PREPARE_CLIMB` → `CLIMB`). |
| **Left stick (click)** | `onTrue(setPoses(CENTER_OF_RIGHT_CORAL_STATION))` | Hard-reset pose to a known coral-station location for vision recovery. |
| **Right stick (click)** | `onTrue(setPoses(CENTER_OF_LEFT_CORAL_STATION))` | Same, opposite station. The "left stick → right station" inversion is intentional: it matches the operator's mental model of "tap the side that the robot is *not* on." |
| **LB** | `whileTrue(pathFindToNearestLeftBranch if has-coral else pathFindToLeftCoralStation)` | Conditional pathfind: if the intake has a coral, go to the nearest left-side scoring branch; otherwise go to the left coral station. The branch is chosen at schedule time based on `intake.barelyHasCoral()`. |
| **RB** | `whileTrue(pathFindToNearestRightBranch if has-coral else pathFindToRightCoralStation)` | Mirror of LB. |
| **LB + RB** | `whileTrue(pathFindToNearestAlgaeOnReef unless has-coral)` | Chord: drop everything and go score / remove algae, *unless* a coral is currently held (which would be lost). |
| **Start** | `onTrue(setClimbState)` | Force `PREPARE_CLIMB`. |

## Operator Bindings

The operator controller (port 1) is the "select target + intake/outtake" surface.

| Button | Effect |
| --- | --- |
| POV ↓ / ↘︎ / ↖︎ / ↑ | `setCurrentAutoLevel(L2 / L1 / L3 / L4)` — the POV directions match the visual height of the reef levels on the dashboard. |
| Left trigger / Right trigger | `whileTrue(setIntakeState(true) / setOuttakeState)`, `onFalse(setIdleState)` — momentary intake / outtake. |
| Left stick / Right stick (click) | `toggleNetState` / `toggleProcessorState`. |
| LB / RB | `toggleAutoLevelCoralState` / `setStowState`. |
| A / B / X / Y | `toggleGroundAlgaeState` / `toggleHighReefAlgaeState` / `toggleLowReefAlgaeState` / `toggleCoralStationState`. |
| Back / Start | Duplicate net / processor toggles (operator convenience). |

Finally, `sysID.configureBindings(opXbox)` adds the operator's Back+Start ↔ X/Y SysID chord triggers (quasi-static and dynamic, forward and reverse) over the four-direction routine matrix. The chord pattern means SysID can only be triggered intentionally — there is no accidental routine launch on a single button.

### Auto-heading + trust-cameras coupling

The A-button chord deserves a note: `toggleAutoHeading` flips the `Swerve.DriveMode` from translating to a heading-locked variant, and `toggleTrustCameras` flips the Localizer's `trustCameras` flag (see [`LOCALIZER.md`](subsystems/LOCALIZER.md)). They are intentionally tied so the driver can't end up in a state where the chassis is locking to a vision target the localizer is rejecting (or vice-versa). The conditional 250 ms rumble exists so the driver has a haptic cue that they entered, not exited, the auto-heading mode without looking at the dashboard.

## `periodic()` and `getAutonomousCommand()`

`periodic()` is called from `Robot.robotPeriodic()` and just forwards to `robotStates.publishValues()`, which dispatches into each subsystem's telemetry class.

`getAutonomousCommand()` returns `autoManager.getFinalAutoCommand()` *without caching* — this is intentional. `AutoManager` recompiles `currentCommand` whenever any SmartDashboard chooser fires `onChange(...)`, so callers must re-fetch on every `autonomousInit()` to get the latest compiled routine.

## Implementation note on tuning

The DogLog tuple, the warmup-command schedule order, and the LB/RB conditional thresholds (`intake.barelyHasCoral()`) were tuned through practice-field iteration. Their *roles* are documented above; their numeric specifics are intentional and stable but season-specific.

## See Also

- [`RobotStates`](ROBOT_STATES.md) — State transitions invoked by these bindings.
- [`Commands`](commands/README.md) — Implementation of the default commands and the `pathFindTo*` helpers.
- [`AutoManager`](autos/AUTO_MANAGER.md) — Source of `getAutonomousCommand()`'s return value.
- [`SysID`](util/OTHER.md) — Chord-triggered characterization routines wired in by `sysID.configureBindings(opXbox)`.
