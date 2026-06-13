# Development Guide

This document provides a generic scaffold for developing the software of an FRC robot — useful when bringing up a new season or onboarding new programmers. For deeper information specific to Rowdy25's architecture, design patterns, and AI-agent integration guidelines, see [AGENTS.md](AGENTS.md).

## Contents

- [AGENTS.md](AGENTS.md) — Comprehensive overview of Rowdy25 architecture, subsystem state machines, data flows, and developer workflows (not for learning, but rather for review or context for an LLM)
- Development steps (below) — Generic FRC development milestones and checklist

## Development Steps

### Step 1: Primitive

- [ ] Codebase (`Robot.java`, `RobotContainer.java`, `Constants.java`)
  - [ ] Utility classes
  - [ ] Vendor libraries imported and verified
- [ ] Framework — subsystems
  - [ ] Motor configurations
  - [ ] Subsystem scaffolds (class declarations, getters, telemetry classes)
- [ ] Framework — vision
  - [ ] Test camera data and brainstorm desired metrics
  - [ ] Optimize localization (offsets, trust filters)
- [ ] Exception: drivetrain and camera data
  - [ ] Feedforward and PID tuning for stable, predictable driving

### Step 2: Operational

- [ ] Basic control
  - [ ] Teleop manual control of every subsystem from joystick axes
  - [ ] Drivetrain-only autonomous paths
- [ ] Framework — commands
  - [ ] Brainstorm primitive commands
  - [ ] Brainstorm secondary / composite commands
  - [ ] Brainstorm interactions between subsystems

### Step 3: Optimization

- [ ] Advanced control
  - [ ] Optimize PID values for clarified position or velocity targets (presets)
  - [ ] Button bindings for more sophisticated / automated actions
- [ ] Advanced vision
  - [ ] Integrate camera data and ML for automated tasks

### Step 4: Autonomous

- [ ] Optimize auto paths with automated commands
- [ ] Vary paths for strategy
- [ ] Integrate camera data / ML to dynamically configure paths

## Rowdy25-Specific Workflow

Map the generic steps above onto Rowdy25's actual structure:

- Review [AGENTS.md](AGENTS.md) for the state-machine design (`RobotStates.java`) and how the superstructure subsystems (Elevator, Pivot, Wrist, Intake, Swerve) coordinate.
- Trace each step to actual Rowdy25 classes: `RobotContainer.java` initialization, `RobotStates.State` enum, `orderedTransition()`, and command composition patterns.
- Account for Reefscape-specific phases: ground coral intake → L1–L4 scoring → algae handling → climb sequence.
- Verify PID and SysID tuning per motor matches the actual robot variant (`constants/variants/`).
