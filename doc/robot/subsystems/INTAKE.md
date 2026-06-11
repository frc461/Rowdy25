# Intake Subsystem
The [Intake](../../src/main/java/io/github/frc461/rowdy25/subsystems/intake/Intake.java) class manages the game piece roller motor and sensor-based detection (CANandcolor proximity, distance sensor).
## States
Supports intake, hold, and outtake positions. Game piece detection via proximity and color sensors enables automatic state transitions.
## Control
- TalonFX roller motor with configurable intake/outtake speeds
- CANandcolor sensor for coral/algae differentiation
- Distance sensor for coral obstruction detection
- Automatic stow when piece detected
## Safety
Detects game piece presence; transitions to safe hold state automatically to prevent jamming or dropping.
## Tuning
Intake/outtake speeds and sensor thresholds in IntakeConstants.
