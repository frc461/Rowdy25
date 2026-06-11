# Getting Started

This guide will help you get started with an in-depth analysis of Team 461's Rowdy25 Robot Software. The goal of this documentation is help build understanding into what robot software development actually entails and how to develop robot software. Below will include helpful advice and best practices when contributing to and deploying our code.

*NOTE: The software and imported libraries for this project may be versioned for FRC 2025. Using this project as a template for a different season or a modern project might require updates.*

## Deployment

Before anything, make sure:

- [WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) is installed
- [Git for Windows](https://git-scm.com/download/win) or [Git (Linux/macOS)](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) is installed
- Network on device is connected to your robot

Now we are going to retrieve the code from this repository to deploy it.

1. Open your terminal and enter the command: `git clone <your-repo-url-for-Rowdy25.git>` or clone the local path if you already have it checked out.
2. Once it finishes cloning, open WPILib VSCode Client (or your IDE) and open the cloned folder as a project.
3. Check the vendor dependency manifests under `vendordeps/` to confirm that vendor libraries are available and configured.

Important: if Gradle appears to override your system Java version you can force the build to use a specific JDK by passing the Gradle property:

```
./gradlew -Dorg.gradle.java.home=/path/to/your/jdk
```

4. Press `ctrl`+`shift`+`P` to open up a VSCode menu, then type in 'Deploy'. Press `enter` (or use `./gradlew deploy` from the command line once your team number and preferences are set).

## Contributing (Version Control)

On any issue/feature that doesn't have a branch yet, create an issue and create a respective branch to work on it. This will help keep the main branch and working tree clean and organized. If a branch already exists for an issue/feature, commit changes relevant to that issue/feature to that branch, and generate a pull request (PR) to the main branch when ready.

## Event Best Practices

#### In the pits:
- Keep a programmer on pit crew to manage robot programming and to speak with judges.
- Make as few changes as possible to the code. If a crucial change is needed, *tune/revise* existing code and avoid writing new code as possible, as malfunctions occur more in newly written software.
- Confine to one laptop if possible. FRC events often restrict network access.
- Write a preflight checklist to *thoroughly evaluate* the robot before each match. Test every subsystem thoroughly.

#### Before/during a match:
- MAKE SURE TO SELECT AN AUTO.
- Check the driver station to ensure that everything including the robot, laptop, and controllers, is connected.
- Record auto in all matches when possible.

## Explore & Learn Through Documentation

The documentation is organized to mirror the project structure. Documentation directories under `robot/` correspond to package structures in `src/main/java/io/github/frc461/rowdy25/`:

- [dev](dev) - Development guide and team conventions
- [lib](lib) - Vendor dependencies and installation
- [robot](robot) - Core robot classes and architecture
  - [autos](robot/autos) - Autonomous routines and path management
  - [commands](robot/commands) - Command-based actions (abstracts subsystems)
  - [constants](robot/constants) - Configuration and field positions
  - [subsystems](robot/subsystems) - Mechanical system implementations
  - [util](robot/util) - Vision, field math, and helper utilities

## Recommended Learning Path

This TODO list progresses from **conceptual/architectural knowledge** → **development workflow** → **high-level abstractions (commands)** → **low-level implementations (subsystems/constants)**.

**Conceptual Foundation:**
- [ ] [Getting Started](README.md) - Deployment and contribution workflow
- [ ] [Robot Overview](robot/README.md) - Robot lifecycle and architecture
- [ ] [Commands Overview](robot/commands/README.md) - How commands abstract subsystem actions

**Development:**
- [ ] [Libraries](lib) - Vendor dependency overview and installation
- [ ] [Development Guide](dev/README.md) - Development milestones and team architecture (including AGENTS.md for system-level reference)

**High-Level Abstractions (How pieces work together):**
- [ ] [RobotStates](robot/ROBOT_STATES.md) - The state machine that coordinates all subsystems
- [ ] [RobotContainer](robot/ROBOT_CONTAINER.md) - Where commands bind to controller triggers and state changes

**Low-Level Implementation Details:**
- [ ] [Subsystems Overview](robot/subsystems/README.md) - Lower-level mechanical control
- [ ] [Individual Subsystem Docs](robot/subsystems) - Drivetrain, Elevator, Pivot, Wrist, Intake, Localizer, Lights
- [ ] [Constants](robot/constants/README.md) - Configuration and tuning framework (includes RobotIdentity variant selection)
- [ ] [Utilities](robot/util) - Vision, field math, control helpers

**Mid-to-Advanced Topics:**
- [ ] [Autonomous System](robot/autos/README.md) - Auto routines, pathfinding, dynamic generation
- [ ] [All Commands](robot/commands) - Detailed command implementations
