# Development Guide

This documentation provides a general scaffold for developers in the process of developing the software of an FRC robot. For information about the Rowdy25 architecture, design patterns, and AI agent integration guidelines, see [AGENTS.md](AGENTS.md).

## Contents

- [AGENTS.md](AGENTS.md) - Comprehensive overview of Rowdy25 architecture, subsystem state machines, data flows, and developer workflows
- Development Steps (below) - Generic FRC development milestones and checklist

## Development Steps

## Step 1: Primitive
- [ ] Codebase (`Robot.java`, `RobotContainer.java`, `Constants.java`)
  - [ ] Utility files
  - [ ] Imported vendor dependencies
- [ ] Framework — Subsystems
  - [ ] Motor configurations
  - [ ] Subsystem scaffold (class declaration, getter functions)
- [ ] Framework — Vision
  - [ ] Test camera data & brainstorm desired metrics for measurement
  - [ ] Optimize localization
- [ ] Exception: Drivetrain, Camera data
  - [ ] Feedforward & PID optimization for effective driving

## Step 2: Operational
- [ ] Basic control
  - [ ] Teleop manual control for all subsystems with joysticks
  - [ ] Drivetrain-only autonomous paths
- [ ] Framework — Commands
  - [ ] Brainstorm primitive commands
  - [ ] Brainstorm secondary/more complex commands
  - [ ] Brainstorm interaction between subsystems

## Step 3: Optimization
- [ ] Advanced control
  - [ ] Optimize PID values for clarified positional or velocity identification, i.e., presets
  - [ ] Button bindings for certain, more sophisticated/automated actions
- [ ] Advanced Vision
  - [ ] Integrate camera data/machine learning to implement automated tasks

## Step 4: Autonomous
- [ ] Optimize auto paths with automated commands
- [ ] Vary paths for strategy
- [ ] Integrate camera data/machine learning to dynamically configure paths

## Rowdy25-Specific Development Workflow

The generic development steps above should be adapted to Rowdy25's specific structure and phases:
- Review [AGENTS.md](AGENTS.md) for the state machine design (`RobotStates.java`) and how subsystems (Elevator, Pivot, Wrist, Intake, Swerve) coordinate
- Map each step to actual Rowdy25 classes: `RobotContainer.java` initialization, `RobotStates` enum transitions, and command composition patterns
- Consider Reefscape-specific phases: ground coral intake → L1-L4 scoring → algae handling → climb sequence
- Verify PID/SysID tuning for each motor subsystem matches actual hardware (constant definitions in `constants/variants/`)
