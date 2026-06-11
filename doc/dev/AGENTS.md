# Rowdy 25 - AI Agent Guidelines

FRC 2025 robot codebase (Team 461 "Westside Robotics"). This document guides AI agents through the architecture, patterns, and workflows specific to this 2025 Reefscape robot.

## Architecture Overview

### Lifecycle & Entry Points
- **`Main.java`** → `Robot.java` → `RobotContainer.java`
- `RobotBase.startRobot()` launches the robot instance
- `Robot` extends `TimedRobot` with periodic methods: `robotPeriodic()`, `autonomousInit/Periodic()`, `teleopInit/Periodic()`
- `RobotContainer` initializes subsystems, commands, auto manager, and controller bindings (single entry point for all setup)

### Core Subsystem Architecture: State Machines

**`RobotStates.java`** (1058 lines) is the integrating superstructure:
- Enum `State` with 25+ states: `STOW`, `L1_CORAL`, `L2_CORAL`, `PROCESSOR`, `CLIMB`, etc.
- Manages coordinated transitions across elevator, pivot, wrist, and intake subsystems
- Each subsystem has parallel state enums (e.g., `Pivot.State.L2_CORAL_AT_BRANCH`)
- Uses `Trigger` system for state-based automation: `stowState.onTrue(...)` chains commands
- Method `orderedTransition()` ensures safe non-conflicting movement (e.g., stow wrist before moving pivot through certain ranges)

**Subsystems** (each has `*Telemetry.java` for logging):
- `Swerve`: Phoenix 6 swerve with LocalADStar pathfinding, multi-camera localization
- `Elevator`, `Pivot`, `Wrist`: TalonFX Kraken motors with PID/SVAG profiles
- `Intake`: Motor + sensor-based coral/algae detection
- `Lights`: LED strips (currently disabled on hardware)

### Key Data Flows

1. **State Transitions**: Controller input → `RobotStates.toggle*State()` → state setter → `Trigger` onTrue → `orderedTransition()` → subsystem position commands
2. **Localization**: Multiple cameras (Limelight, PhotonVision, QuestNav) → `Localizer` → pose estimation with vision-based trust filter
3. **Autonomous**: `AutoManager` (chooser-based) → `PathPlanner` paths → `NamedCommands` event markers trigger state changes
4. **Telemetry**: `DogLog` + `NetworkTable` publishers in each subsystem → SmartDashboard via `Shuffleboard`

## Critical Developer Workflows

### Build & Deploy
```bash
# Standard Gradle build (respects JAVA_HOME; build.gradle specifies Java 17)
./gradlew build

# If Gradle overrides Java version, force your JDK:
./gradlew -Dorg.gradle.java.home=/path/to/your/jdk build

# Deploy to RoboRIO (requires team number in .wpilib/wpilib_preferences.json)
./gradlew deploy

# Simulation with GUI:
./gradlew simulateJava
```

### Testing & Tuning
- **SysID**: `SysID.java` utility class for motor characterization (feedforward tuning)
- **SmartDashboard Choosers**: `RobotStates.stateChooser` allows manual state selection during disabled mode
- **Telemetry**: DogLog logs all subsystem states; check `/media/logs/` on roboRIO

### Robot Identity & Constants
`RobotIdentity.initializeConstants()` (called in Robot constructor) loads variant constants based on MAC address:
- `DefaultConstants` (alpha bot)
- `CompConstants` (competition bot)
- `SimConstants` (simulation)
- `TestConstants` (test bench)

Edit `constants/variants/*.java` for robot-specific PID gains, motor IDs, and presets.

## Project-Specific Conventions

### State Management Pattern
States represent **physical robot positions** + **subsystem coordination intent**:
- `STOW`: Safe resting position, transitions to `L2_L3_L4_STOW` when holding coral for L2+ scoring
- Coral scoring: `GROUND_CORAL` (pickup) → `L1_CORAL` / `L2_CORAL` / `L3_CORAL` / `L4_CORAL` (aim) → `OUTTAKE` (score)
- Algae scoring: `LOW_REEF_ALGAE` / `HIGH_REEF_ALGAE` (pickup) → `PROCESSOR` / `NET` (score)
- Camera trust toggle: `swerve.localizer.trustCameras` enables/disables vision-based positioning (fallback: intake-based outtake)

### Coordinate System & Subsystem Positions
- **Swerve**: Field-centric (blue alliance = 0°, red alliance = 180°)
- **Pivot/Wrist/Elevator**: Motor rotations → degrees/inches via ratio constants in `Constants.java`
- `Localizer.java` manages April Tag-based pose + odometry + vision corrections
- `FieldUtil.java` contains reef branch positions and coral station locations (alliance-flip aware)

### Command Composition Pattern
- **Individual commands** often wrap subsystem state setters: `pivot::setL2CoralState`
- **Ordered sequences** use `InstantCommand().andThen(waitUntil(...)).andThen(...)`
- **Conditional logic** via `ConditionalCommand(ifTrue, ifFalse, condition)` or `.onlyIf(boolean)`
- **Parallel execution** with `.alongWith()` (e.g., intake intaking while pivot moves)
- **Interruption**: `.until(condition)` cancels command chain when condition becomes true

### Telemetry & Logging
- **DogLog**: Deep logging of subsystem states, positions, command traces
- **NetworkTable publishers**: Real-time SmartDashboard updates (e.g., `robotStatesPub.set(currentState.name())`)
- **Servo Hub status**: Tracks ratchet engagement (pivot safety mechanism)
- **Vision debug**: PhotonVision and Limelight targets logged per camera

## Integration Points & External Dependencies

### Vision Systems
- **Limelight**: April Tag pose estimation + alliance flip detection
- **PhotonVision**: Color-based coral/algae detection + object tracking
- **QuestNav**: Backup/alternate SLAM-based localization
- All mounted with specific offsets (`COLOR_FORWARD`, `LL_PITCH`, etc.) in vision constants

### PathPlanner Integration
- **Path files**: `src/main/deploy/pathplanner/paths/` (auto-generated in Pathplanner UI)
- **Named Commands**: Register via `NamedCommands.registerCommand(marker, command)` in `RobotContainer`
- **Event markers**: Paths trigger commands at specific waypoints (e.g., `INTAKE_MARKER` → `setGroundCoralState()`)
- **Pathfinding**: `LocalADStar` pathfinder warm-up on startup; avoids defined obstacles (reef zones, wall boundary)

### Hardware Configuration (Constants)
- **Motor IDs & CAN Bus**: `ElevatorConstants.LEAD_ID`, `PivotConstants.ENCODER_ID`, etc.
- **Sensor Ports**: DIO ports for limit switches and proximity sensors
- **Phoenix 6 Configs**: Motor inversion, current limits, neutral modes (coast/brake)
- **Servo Hub**: Ratchet engagement channels (servo PWM control)

## Patterns to Avoid

1. **Direct motor commands outside state machine**: Always transition via `RobotStates.toggle*State()` or subsystem state setters
2. **Blocking waits**: Use `WaitUntilCommand` / `WaitCommand` with command composition, not `Thread.sleep()`
3. **Modifying subsystem positions mid-state**: State transitions should be atomic; use nested states if needed
4. **Ignoring elevator direction in transitions**: `elevator.goingDown()` determines safe transition path (calls `orderedTransition` parameter)
5. **Camera trust hardcode**: Always reference `swerve.localizer.trustCameras` boolean for fallback logic

## File Structure Quick Reference

```
src/main/java/io/github/frc461/rowdy25/
├── Main.java                           # Entry point
├── Robot.java                          # TimedRobot lifecycle
├── RobotContainer.java                 # Initialization hub
├── RobotStates.java                    # State machine superstructure
├── constants/
│   ├── Constants.java                  # Global + nested class constants
│   ├── RobotIdentity.java              # Multi-robot selector
│   ├── RobotPoses.java                 # Field locations (reef, station)
│   └── variants/                       # Robot-specific configs
├── subsystems/
│   ├── drivetrain/Swerve.java          # Phoenix 6 swerve + localization
│   ├── elevator/Elevator.java          # Linear extension
│   ├── pivot/Pivot.java                # Base rotation (pitch)
│   ├── wrist/Wrist.java                # Upper rotation (pitch) + gravity comp
│   ├── intake/Intake.java              # Game piece grip motor + sensor
│   └── localizer/Localizer.java        # Vision + odometry fusion
├── commands/
│   ├── *Command.java                   # Individual subsystem commands
│   ├── drive/                          # Swerve-specific (pathfinding, auto-align)
│   └── auto/                           # Autonomous-specific (search, follow)
├── autos/
│   └── AutoManager.java                # Chooser-based auto builder
└── util/
    ├── FieldUtil.java                  # Reef pose calculations
    ├── vision/                         # Limelight, PhotonVision utilities
    └── DoubleTrueTrigger.java          # Custom trigger logic
```

## Tips for Code Changes

- **Adding a new state**: Add enum value to `RobotStates.State`, create corresponding subsystem states, register trigger in `configureToggleStateTriggers()`
- **Tuning PID**: Edit `constants/variants/` values; use SysID for feedforward, tune P/I/D empirically on robot
- **Debugging state transitions**: Check `orderedTransition()` logic and subsystem `goingDown()` / `goingThroughStow()` predicates
- **Vision changes**: Update offsets in `Constants.VisionConstants` and `Localizer.java` pose correction logic
- **New autonomous routine**: Create path in PathPlanner, register event markers in `RobotContainer`, test with auto mode disabled first

