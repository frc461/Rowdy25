package frc.robot;

import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.DoubleTrueTrigger;
import frc.robot.util.SysID;
import frc.robot.util.Lights;

public class RobotContainer {
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();
    private final Wrist wrist = new Wrist();

    /* Superstructure */
    private final RobotStates robotStates = new RobotStates();

    private final AutoChooser autoChooser = new AutoChooser(swerve, intake);

    /* Sys ID */
    private final SysID sysID = new SysID(swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Driver Tentative:
     * POV buttons / D-pad:
     * Up: Click - outtake, stow
     * Down: Click - manually temp toggle disable all auto aligning
     * Left: Click - Net algae score state, Click again - Outtake, stow
     * Right: Click - Processor algae score state, Click again - Outtake, stow
     *
     * Triggers:
     * Left: Rotate CCW (hold or double click - FAST)
     * Right: Rotate CW (hold or double click - FAST)
     *
     * Joysticks:
     * Left: Translation
     * Right:
     * Left Button: Reset position to coral left-far side, Hold: Reset gyro
     * Right Button: Reset position to coral right-far side
     *
     * Bumpers:
     * Left: Click - wait until coral is in view then align with then intake coral (ground)
     * Right: Click - wait until algae is in view then align with then intake algae (ground)
     *
     * Buttons:
     *
     * A:
     *     No Coral: Click - Climb state, Click Again - stow slowly
     *     Coral: Click - L4 score state, Click Again - outtake, stow
     *
     * B:
     *     No Coral: Click - Higher algae pickup state, stow automatically, Click Again - stow
     *     Coral: Click - L1 score state, Click Again - outtake, stow
     *
     * X:
     *     No Coral: Click - Lower algae pickup state, stow automatically, Click Again - stow
     *     Coral: Click - L3 score state, Click Again - outtake, stow
     *
     * Y:
     *     No Coral: Click - Coral pickup state, stow automatically, Click Again - Cancel
     *     Coral: Click - L2 score state, Click Again - outtake, stow
     */

    private final static CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / D-pad:
     * Up: Click - L2 score state, Click Again - outtake, stow
     * Down: Click - L4 score state, Click Again - outtake, stow
     * Left: Click - L3 score state, Click Again - outtake, stow
     * Right: Click - L1 score state, Click Again - outtake, stow
     *
     * Triggers:
     * Left: Hold - Outtake
     * Right: Hold - intake
     *
     * Joysticks:
     * Left: Move elevator (x), rotate pivot (y)
     * Right: Rotate wrist
     * Left Button: Click - Net algae score state, Click again - Outtake, stow
     * Right Button: Click - Processor algae score state, Click again - Outtake, stow
     *
     * Bumpers:
     * Left: Click - wait until coral is in view then align with then intake coral (ground)
     * Right: Click - Stow
     * TENTATIVE: Right: Click - wait until algae is in view then align with then intake algae (ground)
     *
     * Buttons:
     * A: Click - Climb state, Click Again - stow slowly
     * B: Click - Higher algae pickup state, stow automatically, Click Again - Stow
     * X: Click - Lower algae pickup state, stow automatically, Click again - Stow
     * Y: Click - Coral pickup state, stow automatically, Click Again - Cancel
     */

    public RobotContainer() {
        setDefaultCommands();
        configureBindings();
        configureToggleStateTriggers(); 

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
                        elevator::getPosition,
                        driverXbox::getLeftY,
                        driverXbox::getLeftX,
                        driverXbox::getLeftTriggerAxis,
                        driverXbox::getRightTriggerAxis,
                        DoubleTrueTrigger.doubleTrue(driverXbox.leftTrigger(0.5), 0.5),
                        DoubleTrueTrigger.doubleTrue(driverXbox.rightTrigger(0.5), 0.5) // TODO TEST: TEST DOUBLE-CLICK
                )
        );

        elevator.setDefaultCommand(new ElevatorCommand(elevator, opXbox::getLeftX, pivot::getPosition, robotStates));

        intake.setDefaultCommand(new IntakeCommand(intake));

        pivot.setDefaultCommand(
                new PivotCommand(pivot, () -> -opXbox.getLeftY(), elevator::getPosition, wrist::getPosition, robotStates)
        );

        wrist.setDefaultCommand(
                new WristCommand(wrist, () -> -opXbox.getRightY(), pivot::getPosition, elevator::getPosition, robotStates)
        );
    }

    private void configureBindings() {
        // IMPORTANT: WHEN BINDING DRIVER BUTTONS, TRIGGERS NEED TO BE ON FALSE ESPECIALLY WITH BINDINGS THAT INITIATE DRIVE AUTOMATION UPON HOLD DEBOUNCE
        // SO FIGURE OUT LOGIC CORRECTLY AND CAREFULLY
        // TODO: ON FALSE TRIGGERS FOR DRIVER ONLY BINDINGS

        driverXbox.povUp().onFalse(new InstantCommand(robotStates::setOuttakeState));
        driverXbox.povDown().onFalse(new InstantCommand(swerve::toggleAutoHeading));
        driverXbox.povLeft().onFalse(new InstantCommand(robotStates::toggleNetState));
        driverXbox.povRight().onFalse(new InstantCommand(robotStates::toggleProcessorState));

        driverXbox.leftStick().onFalse(new InstantCommand(() -> swerve.localizer.setPoses(Constants.FAR_LEFT_CORAL_STATION)));
        driverXbox.rightStick().onFalse(new InstantCommand(() -> swerve.localizer.setPoses(Constants.FAR_RIGHT_CORAL_STATION)));

        driverXbox.leftBumper().onFalse(new InstantCommand(robotStates::toggleGroundCoralState));
        driverXbox.rightBumper().onFalse(new InstantCommand(robotStates::toggleGroundAlgaeState));

        driverXbox.a().onFalse(new ConditionalCommand(
                new InstantCommand(robotStates::toggleL4CoralState),
                Commands.none(),
                intake::hasCoral
        ));
        driverXbox.a().debounce(1.5).whileTrue(new ConditionalCommand(
                swerve.pathFindToNearestBranch(elevator::getPosition),
                Commands.none(),
                intake::hasCoral
        ));

        driverXbox.b().onFalse(new ConditionalCommand(
                new InstantCommand(robotStates::toggleL1CoralState),
                new InstantCommand(robotStates::toggleHighReefAlgaeState),
                intake::hasCoral
        ));
        driverXbox.b().debounce(1.5).whileTrue(new ConditionalCommand(
                swerve.pathFindToNearestBranch(elevator::getPosition),
                Commands.none(),
                intake::hasCoral
        ));

        driverXbox.x().onFalse(new ConditionalCommand(
                new InstantCommand(robotStates::toggleL3CoralState),
                new InstantCommand(robotStates::toggleLowReefAlgaeState),
                intake::hasCoral
        ));
        driverXbox.x().debounce(1.5).whileTrue(new ConditionalCommand(
                swerve.pathFindToNearestBranch(elevator::getPosition),
                Commands.none(),
                intake::hasCoral
        ));

        driverXbox.y().onFalse(new ConditionalCommand(
                new InstantCommand(robotStates::toggleL2CoralState),
                new InstantCommand(robotStates::toggleCoralStationState),
                intake::hasCoral
        ));
        driverXbox.y().debounce(1.5).whileTrue(new ConditionalCommand(
                swerve.pathFindToNearestBranch(elevator::getPosition),
                Commands.none(),
                intake::hasCoral
        ));

        opXbox.povRight().onTrue(new InstantCommand(robotStates::toggleL1CoralState));
        opXbox.povUp().onTrue(new InstantCommand(robotStates::toggleL2CoralState));
        opXbox.povLeft().onTrue(new InstantCommand(robotStates::toggleL3CoralState));
        opXbox.povDown().onTrue(new InstantCommand(robotStates::toggleL4CoralState));

        opXbox.leftTrigger().onTrue(new InstantCommand(intake::setOuttakeState).andThen(new WaitUntilCommand(() -> !opXbox.leftTrigger().getAsBoolean())).andThen(intake::setIdleState));
        opXbox.rightTrigger().onTrue(new InstantCommand(intake::setIntakeState).andThen(new WaitUntilCommand(() -> !opXbox.a().getAsBoolean())).andThen(intake::setIdleState));

        opXbox.leftStick().onTrue(new InstantCommand(robotStates::toggleNetState));
        opXbox.rightStick().onTrue(new InstantCommand(robotStates::toggleProcessorState));

        opXbox.leftBumper().onTrue(new InstantCommand(robotStates::toggleGroundCoralState));
//        opXbox.rightBumper().onTrue(new InstantCommand(robotStates::toggleGroundAlgaeState));
        opXbox.rightBumper().onTrue(new InstantCommand(robotStates::setStowState));

        opXbox.a().onTrue(Commands.none());
        opXbox.b().onTrue(new InstantCommand(robotStates::toggleHighReefAlgaeState));
        opXbox.x().onTrue(new InstantCommand(robotStates::toggleLowReefAlgaeState));
        opXbox.y().onTrue(new InstantCommand(robotStates::toggleCoralStationState));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(opXbox);
    }

    private void configureToggleStateTriggers() {
        robotStates.configureToggleStateTriggers(swerve, elevator, intake, pivot, wrist);
    }

    public void periodic() {
        robotStates.publishValues();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
