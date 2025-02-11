package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.SysID;
import frc.robot.util.Lights;

public class RobotContainer {
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();
    private final Elevator elevator = new Elevator();
    private final Wrist wrist = new Wrist();
    
    private final AutoChooser autoChooser = new AutoChooser(swerve);

    /* Sys ID */
    private final SysID sysID = new SysID(swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Currently Allocated For Driver:
     * POV buttons / D-pad:
     * Up: Rotate then translate to a game element (if applicable)
     * Down:
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
     * Right Button:
     *
     * Bumpers:
     * Left: Tag alignment
     * Right: Game element alignment
     *
     * Buttons:
     *
     * A: Manual-configure Quest (if applicable)
     *
     * B: Toggle localization strategy
     *
     * X:
     *
     * Y: Reset gyro
     */

    /* Driver Tentative:
     * POV buttons / D-pad:
     * Up:
     * Down:
     * Left: Click - manually temp toggle between lower and higher algae reef intake level
     * Right: Click - manually temp toggle between processor and net height
     *
     * Triggers:
     * Left: Rotate CCW (with bumper - FAST)
     * Right: Rotate CW (with bumper - FAST)
     *
     * Joysticks:
     * Left: Translation
     * Right:
     * Left Button: Reset position to coral left side, Hold: Reset gyro
     * Right Button: Reset position to coral right side
     *
     * Bumpers:
     * Left: Hold - Align with then intake coral (ground)
     * Right: Hold - Align with then intake algae (ground)
     *
     * Buttons:
     *
     * A:
     *     No Coral: Click - Climb stage, Click Again - stow slowly
     *     Coral: Click - L4 score stage, Click Again - outtake, stow
     *
     * B:
     *     No Coral: Click - Algae pickup stage (height determined by heading, then camera, then stow automatically), stow automatically, Click Again - Stow
     *     Coral: Click - L1 score stage, Click Again - outtake, stow
     *
     * X:
     *     No Coral: Click - Coral pickup stage, stow automatically, Click Again - Cancel
     *     Coral: Click - L3 score stage, Click Again - outtake, stow
     *
     * Y:
     *     No Coral: Click - Algae score stage (processor height if within 45 degrees of processor, otherwise net height), Click again - Outtake, stow
     *     Coral: Click - L2 score stage, Click Again - outtake, stow
     */

    private final static CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / D-pad:
     * Up: Click - L2 score stage, Click Again - outtake, stow
     * Down: Click - L4 score stage, Click Again - outtake, stow
     * Left: Click - L3 score stage, Click Again - outtake, stow
     * Right: Click - L1 score stage, Click Again - outtake, stow
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
     * Left: Click - manually temp toggle between lower and higher algae reef intake level
     * Right: Click - manually temp toggle between processor and net height
     *
     * Buttons:
     * A: Click - Climb stage, Click Again - stow slowly
     * B: Click - Algae pickup stage (height determined by heading, then camera, then stow automatically), stow automatically, Click Again - Stow
     * X: Click - Coral pickup stage, stow automatically, Click Again - Cancel
     * Y: Click - Algae score stage (processor height if within 45 degrees of processor, otherwise net height), Click again - Outtake, stow
     */

    public RobotContainer() {
        setDefaultCommands();
        configureBindings();

        Lights.configureLights();

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
                        () -> driverXbox.rightBumper().getAsBoolean(),
                        () -> driverXbox.x().getAsBoolean(),
                        () -> driverXbox.y().getAsBoolean()
                )
        );

        intake.setDefaultCommand(new IntakeCommand(intake));

        // TODO SHOP: TEST AXES
        elevator.setDefaultCommand(
                new ElevatorCommand(
                        elevator,
                        () -> opXbox.getRightTriggerAxis() - opXbox.getLeftTriggerAxis(),
                        pivot::getPosition
                )
        );

        pivot.setDefaultCommand(
                new PivotCommand(pivot, () -> -opXbox.getLeftY(), elevator::getPosition, wrist::getPosition)
        );

        wrist.setDefaultCommand(
                new WristCommand(wrist, () -> -opXbox.getRightY(), pivot::getPosition)
        );
    }

    private void configureBindings() {

        driverXbox.a().onTrue(new InstantCommand(swerve.localizer::configureQuestOffset));

        driverXbox.b().onTrue(new InstantCommand(swerve.localizer::toggleLocalizationStrategy));

        // reset the field-centric heading on left joystick press
        driverXbox.leftStick().onTrue(new InstantCommand(swerve::resetGyro));

        driverXbox.povLeft().whileTrue(swerve.directMoveToObject());

        driverXbox.povRight().whileTrue(swerve.pathFindToNearestBranch());

        // test presets on the operator xbox controller before setting final bindings
        // FOR OPERATOR PRACTICE, JUST USE THE TWO JOYSTICKS & THE TRIGGERS (LT+RT) FOR MOVING PIVOT, ELEVATOR, WRIST, AND INTAKE

        opXbox.rightBumper().onTrue(new InstantCommand(intake::toggleIntakeState));
        opXbox.leftBumper().onTrue(
                new InstantCommand(intake::toggleOuttakeState)
                        .andThen(new WaitUntilCommand(intake::isIdle))
                        .andThen(pivot::setStowState)
                        .andThen(new WaitUntilCommand(pivot::isAtTarget))
                        .andThen(elevator::setStowState)
                        .andThen(new WaitUntilCommand(elevator::isAtTarget))
                        .andThen(wrist::setStowState)
        );

//        opXbox.povLeft().onTrue(new InstantCommand(elevator::toggleL2CoralState));
//        opXbox.povLeft().onTrue(new InstantCommand(pivot::toggleL2CoralState));
//        opXbox.povLeft().onTrue(new InstantCommand(wrist::toggleGroundCoralState));

//        opXbox.povUp().onTrue(new InstantCommand(elevator::toggleL4CoralState));
//        opXbox.povUp().onTrue(new InstantCommand(pivot::toggleL4CoralState));
//        opXbox.povUp().onTrue(new InstantCommand(wrist::toggleGroundAlgaeState));

//        opXbox.povRight().onTrue(new InstantCommand(elevator::toggleL3CoralState));
//        opXbox.povRight().onTrue(new InstantCommand(pivot::toggleL3CoralState));
//        opXbox.povRight().onTrue(new InstantCommand(wrist::toggleCoralStationState));

//        opXbox.povDown().onTrue(new InstantCommand(elevator::setStowState));
//        opXbox.povDown().onTrue(new InstantCommand(pivot::setStowState));
//        opXbox.povDown().onTrue(new InstantCommand(pivot::toggleRatchet));
//        opXbox.povDown().onTrue(new InstantCommand(wrist::setStowState));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(opXbox);
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
