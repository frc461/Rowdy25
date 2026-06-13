# Getting Started

This guide will help you get started with an in-depth study of Team 461's Rowdy25 robot software. The goal of this documentation is to build understanding of what FRC robot software development actually entails and how to develop robot software effectively. Below are helpful tips and best practices for contributing to and deploying this code.

> **Note:** The software and imported libraries in this project are versioned for FRC 2025 (Reefscape). Using this project as a template for a different season or modern release will require updating WPILib, vendor libraries, and possibly Java version targets.

## Deployment

Before anything, make sure:

- [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) is installed
- [Git for Windows](https://git-scm.com/download/win) or [Git (Linux/macOS)](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) is installed
- The network on your laptop is configured for your robot's radio / RoboRIO

To retrieve and deploy the code:

1. Clone the repository: `git clone <your-repo-url-for-Rowdy25.git>` (or open an existing local checkout).
2. Open the cloned folder as a project in WPILib VSCode (or your preferred IDE).
3. Check `vendordeps/` against [Vendor Libraries](lib/VENDOR_LIBRARIES.md) to confirm all vendor library manifests are present.
4. If Gradle picks up an unintended Java version (this project targets Java 17), force the build to use a specific JDK with:

   ```bash
   ./gradlew -Dorg.gradle.java.home=/path/to/your/jdk build
   ```

5. Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> in WPILib VSCode and run **"WPILib: Deploy Robot Code"** (or from the command line: `./gradlew deploy` once your team number is set in `.wpilib/wpilib_preferences.json`).

To run the simulator: `./gradlew simulateJava`.

## Contributing (Version Control)

For any issue or feature that doesn't already have a branch, open an issue first and create a branch off it. This keeps `main` and the working tree clean and organized. If a branch already exists for the work you're doing, commit there and open a pull request back to `main` when ready.

## Event Best Practices

### In the pits

- Keep a programmer on pit crew to manage robot code and to speak with judges.
- Make as few code changes as possible. If a change is required, *tune or revise* existing code rather than writing new code — bugs are far more likely in freshly written code.
- Confine deploys to a single laptop when possible. Event networks often restrict internet access, and a single source of truth avoids deploying stale revisions.
- Maintain a preflight checklist to thoroughly evaluate the robot before each match. Test every subsystem.

### Before / during a match

- **Make sure to select an auto routine before the match starts.**
- Verify on the driver station that the robot, laptop, and controllers are all connected.
- Record auto in every match when possible.

## Explore & Learn Through Documentation

The documentation is organized to mirror the project structure. Directories under `robot/` correspond to packages under `src/main/java/io/github/frc461/rowdy25/`:

- [dev](dev) — Development guide and team conventions
- [lib](lib) — Vendor dependencies and installation
- [robot](robot) — Core robot classes and architecture
  - [autos](robot/autos) — Autonomous routines and path management
  - [commands](robot/commands) — Command-based actions (abstractions over subsystems)
  - [constants](robot/constants) — Configuration and field positions
  - [subsystems](robot/subsystems) — Mechanical system implementations
  - [util](robot/util) — Vision, field math, and helper utilities

## Recommended Learning Path

This checklist progresses from **conceptual / architectural knowledge** → **development workflow** → **high-level abstractions (commands)** → **low-level implementation details**.

**Conceptual foundation:**

- [ ] [Getting Started](README.md) — Deployment and contribution workflow (this file)
- [ ] [Robot Overview](robot/README.md) — Robot overview
- [ ] [Commands Overview](robot/commands/README.md) — How commands abstract subsystem actions
- [ ] [Robot Class](robot/ROBOT.md) - Robot lifecycle and architecture

**Development:**

- [ ] [Libraries](lib) — Vendor dependency overview and installation
- [ ] [Development Guide](dev/README.md) — Development milestones and team architecture (see also [AGENTS.md](dev/AGENTS.md) for a system-level reference)

**High-level abstractions (how pieces fit together):**

- [ ] [RobotStates](robot/ROBOT_STATES.md) — State machine that coordinates the superstructure
- [ ] [RobotContainer](robot/ROBOT_CONTAINER.md) — Where commands bind to controller triggers and state changes

**Low-level implementation details:**

- [ ] [Subsystems Overview](robot/subsystems/README.md) — Lower-level mechanical control
- [ ] [Individual Subsystem Docs](robot/subsystems) — Drivetrain, Elevator, Pivot, Wrist, Intake, Localizer, Lights
- [ ] [Constants](robot/constants/README.md) — Configuration and tuning framework (including [`RobotIdentity`](robot/constants/ROBOT_IDENTITY.md) variant selection)
- [ ] [Utilities](robot/util) — Vision, field math, and control helpers

**Mid-to-advanced topics:**

- [ ] [Autonomous System](robot/autos/README.md) — Auto routines, pathfinding, dynamic generation
- [ ] [All Commands](robot/commands) — Detailed command implementations
