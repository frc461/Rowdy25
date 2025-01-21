package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.autos.AutoChooser;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.SysID;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class RobotContainer {
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();
    private final Wrist wrist = new Wrist();

    private final AutoChooser autoChooser = new AutoChooser(swerve);

    /* Sys ID */
    private final SysID sysID = new SysID(swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Currently Allocated For Driver:
     * POV buttons / D-pad:
     * Up: 
     *  Current: Rotate then translate to a game element (if applicable)
     * Down: Climb?
     * Left:
     * Right:
     *
     * Triggers:
     * Left: Rotate CCW (with bumper - FAST)
     * Right: Rotate CW (with bumper - FAST)
     *
     * Joysticks:
     * Left: Translation
     * Right:
     * Left Button:
     * Right Button: intake coral from station (maybe also auto-align to 2 different pickup locations based on proximity)
     *
     * Bumpers:
     * Left: chase & intake algae from ground 
     *  current: Tag alignment
     * Right: chase & intake coral from ground    
     *  current: Game element alignment
     *
     * Buttons:
     * Note: All buttons also align to closest target rotationally & translationally using tags/piece detection. They also outtake on release if necessary and all go back to stow position.
     *
     * A: L1 coral height/angle if coral in intake, processor otherwise  // TODO THINK OF ONE DRIVER AUTOMATED TASKS WITH BUTTONS (MIGHT BE VERY COMPLICATED)
     *  current: Manual-configure Quest (if applicable)
     *
     * B: L2 coral height/angle if coral in intake, intake low algae from reef otherwise
     *  current: Toggle localization strategy
     *
     * X: L3 coral height/angle if coral in intake, intake high algae from reef otherwise
     *
     * Y: L4 coral height/angle if coral in intake, net otherwise
     *  current: Reset gyro
     */

    private final static CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / D-pad:
     * Up:
     * Down:
     * Left:
     * Right:
     *
     * Triggers:
     * Left: Rotate wrist down
     * Right: Rotate wrist up
     *
     * Joysticks:
     * Left: Rotate elevator
     * Right: Rotate pivot
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
        setDefaultCommands();
        configureBindings();
       
        // DogLogOptions(BooleanSupplier ntPublish, boolean captureNt, boolean captureDs, boolean logExtras, boolean captureConsole, int logEntryQueueCapacity)
        DogLog.setOptions(new DogLogOptions(() -> false, false, true, true, false, 5000));
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

        elevator.setDefaultCommand(
                new RunCommand(
                        () -> elevator.moveElevator(MathUtil.applyDeadband(-opXbox.getLeftY(), Constants.DEADBAND)),
                        elevator
                )
        );

        pivot.setDefaultCommand(
                new RunCommand(
                        () -> pivot.movePivot(MathUtil.applyDeadband(-opXbox.getRightY(), Constants.DEADBAND)),
                        pivot
                )
        );

        wrist.setDefaultCommand(
                new RunCommand(
                        () -> wrist.moveWrist(MathUtil.applyDeadband(opXbox.getRightTriggerAxis() - opXbox.getLeftTriggerAxis(), Constants.DEADBAND)),
                        wrist
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
