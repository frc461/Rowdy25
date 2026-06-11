# RobotIdentity Class
[RobotIdentity](../../src/main/java/io/github/frc461/rowdy25/constants/RobotIdentity.java) selects robot-specific constants at startup by detecting the robot's MAC address.
## Variants
- **DefaultConstants** - Alpha/prototype bot
- **CompConstants** - Competition bot (fully tuned)
- **SimConstants** - WPILib simulation environment
- **TestConstants** - Bench testing of subsystems
## Initialization
RobotIdentity.initializeConstants() is called in the Robot constructor. It reads the RoboRIO's MAC address via MacAddress.getMacAddress() and loads the matching variant's constants.
## Adding New Robots
To register a new robot:
1. Create a new variant MAC constant
2. Create a variant constants class extending the base variant
3. Add the mapping in initializeConstants()
## See Also
- [Constants](CONSTANTS.md) - Structure of constants
- Variant classes under src/main/java/io/github/frc461/rowdy25/constants/variants/
"@ | Out-File -FilePath 'C:\Users\eugen\Projects\Rowdy25\doc\robot\constants\ROBOT_IDENTITY.md' -Encoding UTF8 -Force
@"
# RobotPoses Class
[RobotPoses](../../src/main/java/io/github/frc461/rowdy25/constants/RobotPoses.java) defines all field landmark poses for the Reefscape game.
## Landmarks
- **Reef Branches** - Positions for L1-L4 coral scoring
- **Coral Station** - Pickup location for coral game pieces
- **Algae Targets** - Reef and processor locations for algae
- **Processor** - Algae scoring location
- **Net** - High algae scoring target
- **Barge** - Climb starting position
## Alliance Awareness
All poses are defined in blue alliance coordinates. FlippingUtil automatically flips poses for red alliance via DriverStation alliance detection.
## Usage
Used by [Pathfinder](../autos/PATHFINDER.md) and [FieldUtil](../util/OTHER.md) for autonomous pathfinding and manual control target calculations.
## Tuning
Verify poses match actual field layout during competition setup.
