# Elevator Subsystem
The [Elevator](../../src/main/java/io/github/frc461/rowdy25/subsystems/elevator/Elevator.java) class manages vertical extension via a TalonFX Kraken motor with leader-follower configuration and Motion Magic Expo voltage control.
## States
Defines 14+ positions: STOW, L2_L3_L4_STOW, CORAL_STATION, GROUND_CORAL/ALGAE, L1/L2/L3/L4_CORAL (at/near branch), LOW/HIGH_REEF_ALGAE, PROCESSOR, NET, PREPARE_CLIMB, CLIMB.
## Control
- Motion Magic Expo for smooth position tracking
- Gravity feedforward compensation
- Limit switch homing for calibration
- Soft limits to prevent mechanical damage
## Tuning
PID and feedforward gains in ElevatorConstants. Use SysID to characterize motor behavior before field deployment.
## See Also
- [ElevatorCommand](../commands/SUBSYSTEM_COMMANDS.md) - Higher-level command interface
- [RobotStates](../ROBOT_STATES.md) - State transitions invoking elevator positions
