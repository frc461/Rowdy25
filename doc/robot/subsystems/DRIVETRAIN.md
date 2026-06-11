# Drivetrain (Swerve) Subsystem
The [Swerve](../../src/main/java/io/github/frc461/rowdy25/subsystems/drivetrain/Swerve.java) class extends CTRE's [SwerveDrivetrain](https://api.ctr-electronics.com/phoenix6/release/java/) and manages omnidirectional movement via four SDS MK4i swerve modules.
## Core Functionality
- **Drive Modes** - IDLE, ROTATING, TRANSLATING, AUTO_HEADING (target angle tracking)
- **Localization** - Integrates [Localizer](LOCALIZER.md) for field-relative pose estimation
- **Pathfinding** - Commands for dynamic pathfinding to reef/coral/algae targets via PathPlanner's LocalADStar
- **Default Command** - Joystick drive with field-centric heading control
## Control Methods
- setTeleopDriveCommand() - Manual joystick-based movement
- pathfinddToPoseCommand() - Autonomous pathfinding to target pose
- directMoveToPoseCommand() - Direct movement without pathfinding
- holdAngleCommand() - Maintain heading angle
## Integration
- Motor control via Phoenix 6 TalonFX (drive) and Phoenix Kraken motors (rotation)
- CANcoder absolute encoders for wheel state measurement
- Integrates with PathPlanner via AutoBuilder and LocalADStar
