package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.autos.AutoChooser;
import frc.robot.constants.RobotIdentity;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.SysID;

public class RobotContainer {
    /* Subsystems */
    public final Swerve swerve = new Swerve();

    private final AutoChooser autoChooser = new AutoChooser(swerve);

    /* Sys ID */
    public final SysID sysID = new SysID(swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Currently Allocated For Driver:
     * POV buttons / D-pad:
     * Up:
     * Down:
     * Left:
     * Right:
     *
     * Triggers:
     * Left: "speaker tag" align
     * Right: note align
     *
     * Joysticks:
     * Left: Translation
     * Right: Rotation
     * Left Button:
     * Right Button:
     *
     * Bumpers:
     * Left:
     * Right:
     *
     * Buttons:
     * A: X mode
     * B: Point directions
     * X: recalibrate
     * Y: Re-zero gyro
     */

    public final static CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / D-pad:
     * Up:
     * Down:
     * Left:
     * Right:
     *
     * Triggers:
     * Left:
     * Right:
     *
     * Joysticks:
     * Left:
     * Right:
     *
     * Bumpers:
     * Left:
     * Right:
     *
     * Buttons:
     * A:
     * B:
     * X:
     * Y:
     */

    public RobotContainer() {
        RobotIdentity.initializeConstants();
        setDefaultCommands();
        configureBindings();
       
        DogLog.setOptions(new DogLogOptions(false, false, true, true, false, 5000));
        DogLog.setPdh(new PowerDistribution());

        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
    }

    /* Each subsystem will execute their corresponding command periodically */
    private void setDefaultCommands() {
        /* Note that X is defined as forward according to WPILib convention,
        and Y is defined as to the left according to WPILib convention.
        drive forward with left joystick negative Y (forward),
        drive left with left joystick negative X (left),
        rotate counterclockwise with right joystick negative X (left) */
        swerve.setDefaultCommand(
                swerve.driveFieldCentric(
                        driverXbox::getLeftY,
                        driverXbox::getLeftX,
                        driverXbox::getLeftTriggerAxis,
                        driverXbox::getRightTriggerAxis,
                        () -> driverXbox.leftBumper().getAsBoolean(),
                        () -> driverXbox.rightBumper().getAsBoolean()
                )
        );
    }

    private void configureBindings() {

        driverXbox.a().whileTrue(swerve.runOnce(swerve.localizer::configureQuestOffset));

        // toggle between robot choosing quest nav pose and pose estimation with cameras
        driverXbox.b().onTrue(swerve.runOnce(swerve.localizer::toggleLocalizationStrategy));

        // reset the field-centric heading on y press
        driverXbox.y().onTrue(swerve.resetGyro());

        driverXbox.povUp().whileTrue(swerve.moveToNote());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(driverXbox);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
