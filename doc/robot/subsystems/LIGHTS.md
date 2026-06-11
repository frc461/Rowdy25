# Lights Subsystem
The [Lights](../../src/main/java/io/github/frc461/rowdy25/subsystems/Lights.java) class manages PWM-controlled LED strip indicators (currently disabled on 2025 hardware).
## Purpose
Provides visual feedback for robot state: stow (green), scoring (red), climb (blue), etc.
## Implementation
When re-enabled, subscribes to RobotStates to drive LED color based on current state.
## Status
Hardware disabled for 2025; code remains for future deployment.
