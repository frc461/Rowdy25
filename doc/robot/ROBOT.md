# Robot Class
The [Robot](../../src/main/java/io/github/frc461/rowdy25/Robot.java) class extends [TimedRobot](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/TimedRobot.html) and serves as the main entry point for the robot's control loop.
## Initialization
The [Robot](../../src/main/java/io/github/frc461/rowdy25/Robot.java) constructor:
- Initializes robot constants based on robot identity (MAC address detection)
- Forwards network ports for vision systems (PhotonVision and Limelight)
- Instantiates the RobotContainer
## Lifecycle Methods
- [obotPeriodic()](../../src/main/java/io/github/frc461/rowdy25/Robot.java) - Runs every 20ms regardless of mode; executes command scheduler
- [utonomousInit()](../../src/main/java/io/github/frc461/rowdy25/Robot.java) - Runs once at start of autonomous; schedules selected auto command
- [utonomousPeriodic()](../../src/main/java/io/github/frc461/rowdy25/Robot.java) - Runs periodically during autonomous (not necessary with command scheduling)
- [	eleopInit()](../../src/main/java/io/github/frc461/rowdy25/Robot.java) - Runs once at start of teleop
- [	eleopPeriodic()](../../src/main/java/io/github/frc461/rowdy25/Robot.java) - Runs periodically during teleop (not necessary with command scheduling)
## Command Scheduler
The WPILib [CommandScheduler](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/CommandScheduler.html) is executed in obotPeriodic() to schedule and run all registered commands. Commands are automatically added/removed based on button presses, state transitions, and completion conditions.
