# Pivot Subsystem
The [Pivot](../../src/main/java/io/github/frc461/rowdy25/subsystems/pivot/Pivot.java) class manages base rotation (pitch) via TalonFX with CANcoder absolute encoder and motion magic control.
## States
Supports 10+ states for coral/algae intake and scoring alignment. Includes gravity compensation tuning per load (empty/loaded).
## Safety
Servo hub ratchet engagement mechanism holds position during  disabled/coast modes. Software validates safe transition sequences via RobotStates.orderedTransition().
## Control
- Motion Magic for velocity-limited rotation
- Gravity feedforward (tuned per load state)
- CANcoder offset calibration for absolute position tracking
- Soft limits for mechanical bounds
## Tuning
Constants in PivotConstants. Gravity gains require field characterization with actual load.
