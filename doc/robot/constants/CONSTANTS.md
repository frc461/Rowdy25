# Constants Class
[Constants](../../src/main/java/io/github/frc461/rowdy25/constants/Constants.java) centralizes all robot configuration values in nested static classes by subsystem.
## Nested Classes
- **ElevatorConstants** - Motor IDs, position limits, PID/feedforward gains
- **PivotConstants** - Motor ID, encoder offset, rotation limits, gravity tuning
- **WristConstants** - Motor ID, position presets, control gains
- **IntakeConstants** - Motor ID, roller speeds, sensor ports and thresholds
- **SwerveConstants** - Module IDs, wheel radius, max velocity/acceleration, PathPlanner config
- **AutoConstants** - Path constraints, default velocities
- **VisionConstants** - Camera offsets, field dimensions, sensor DIO ports
## Robot Variants
Actual values are defined in subclasses of the variant classes (DefaultConstants, CompConstants, SimConstants, TestConstants). [RobotIdentity](ROBOT_IDENTITY.md) selects the variant at startup based on MAC address.
## Usage
Access constants via Constants.ElevatorConstants.STOW_POSITION or Constants.SwerveConstants.MAX_VELOCITY, etc.
## Tuning
Edit the appropriate variant class to tune for that specific robot. Use SysID to characterize motors before field deployment.
