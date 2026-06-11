# Lights Subsystem

The [Lights](../../src/main/java/io/github/frc461/rowdy25/subsystems/Lights.java) class wraps PWM-controlled LED strip indicators used for visual feedback about robot state. The hardware was disabled on the 2025 competition robot, but the subsystem is left in place so it can be reactivated without changes elsewhere.

## Purpose

Provides at-a-glance feedback to drivers and human pickup partners about the current superstructure state — stow, scoring, intake-ready, algae held, climb, etc.

## Implementation

When re-enabled, `Lights` reacts to [RobotStates](../ROBOT_STATES.md) transitions to drive LED color and pattern.

## Status

Hardware disabled for 2025; the code remains for future deployment.
