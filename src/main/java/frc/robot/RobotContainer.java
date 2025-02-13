package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Pivot pivot = new Pivot();
    private final Wrist wrist = new Wrist();

    /* Superstructure */
    private final RobotStates robotStates = new RobotStates();

    private final AutoChooser autoChooser = new AutoChooser(swerve);

    /* Sys ID */
    private final SysID sysID = new SysID(swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Driver Tentative:
     * POV buttons / D-pad:
     * Up:
     * Down: Click - manually temp toggle disable all auto aligning
     * Left: Click - manually temp toggle between lower and higher algae reef intake level
     * Right: Click - manually temp toggle between processor and net height
     *
     * Triggers:
     * Left: Rotate CCW (with bumper - FAST)
     * Right: Rotate CW (with bumper - FAST)
     *
     * Joysticks:
     * Left: Translation
     * Right: Flick up - outtake then stow, Flick down - stow
     * Left Button: Reset position to coral left side, Hold: Reset gyro
     * Right Button: Reset position to coral right side
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
     *     No Coral: Click - Algae pickup state (height determined by nearest reef, then camera, then stow automatically), stow automatically, Click Again - Stow
     *     Coral: Click - L1 score state, Click Again - outtake, stow
     *
     * X:
     *     No Coral: Click - Coral pickup state, stow automatically, Click Again - Cancel
     *     Coral: Click - L3 score state, Click Again - outtake, stow
     *
     * Y:
     *     No Coral: Click - Algae score state (net or processor, whichever is closer), Click again - Outtake, stow
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
     * Left: Outtake
     * Right: Stow
     *
     * Joysticks:
     * Left: Move elevator (x), rotate pivot (y)
     * Right: Rotate wrist
     * Left Button: Click - manually temp toggle between lower and higher algae reef intake level
     * Right Button: Click - manually temp toggle between processor and net height
     *
     * Bumpers:
     * Left: Click - wait until coral is in view then align with then intake coral (ground)
     * Right: Click - wait until algae is in view then align with then intake algae (ground)
     *
     * Buttons:
     * A: Click - Climb state, Click Again - stow slowly
     * B: Click - Algae pickup state (height determined by heading, then camera, then stow automatically), stow automatically, Click Again - Stow
     * X: Click - Coral pickup state, stow automatically, Click Again - Cancel
     * Y: Click - Algae score state (processor height if within 45 degrees of processor, otherwise net height), Click again - Outtake, stow
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

        driverXbox.povUp().onTrue(new InstantCommand(robotStates::setOuttakeState));

        // (set to ground state and if holding then wait until there's an object then drive to it)
//        driverXbox.leftBumper().negate().and(driverXbox.leftTrigger().negate()).onTrue(
//                new InstantCommand(robotStates::toggleGroundCoralState)
//        );
//        driverXbox.leftBumper().and(driverXbox.leftTrigger()).onTrue(new InstantCommand(robotStates::setStowState));

        driverXbox.b().onTrue(new InstantCommand(
                () -> robotStates.toggleNearestReefAlgaeState(swerve.localizer.nearestAlgaeIsHigh())
        ));

        opXbox.povRight().onTrue(new InstantCommand(robotStates::toggleL1CoralState));
        opXbox.povUp().onTrue(new InstantCommand(robotStates::toggleL2CoralState));
        opXbox.povLeft().onTrue(new InstantCommand(robotStates::toggleL3CoralState));
        opXbox.povDown().onTrue(new InstantCommand(robotStates::toggleL4CoralState));

        opXbox.leftTrigger().onTrue(new InstantCommand(robotStates::setOuttakeState));
        opXbox.rightTrigger().onTrue(new InstantCommand(robotStates::setStowState));

        opXbox.leftBumper().onTrue(new InstantCommand(robotStates::toggleGroundCoralState));
        opXbox.rightBumper().onTrue(new InstantCommand(robotStates::toggleGroundAlgaeState));

        opXbox.b().onTrue(new InstantCommand(
                () -> robotStates.toggleNearestReefAlgaeState(swerve.localizer.nearestAlgaeIsHigh())
        ));

        opXbox.y().onTrue(new InstantCommand(() -> robotStates.toggleNearestAlgaeScoringLocation(swerve.localizer.nearestAlgaeScoringIsNet())));

        opXbox.x().onTrue(new InstantCommand(robotStates::toggleCoralStationState));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(opXbox);
    }

    private void configureToggleStateTriggers() {
        robotStates.configureToggleStateTriggers(swerve, elevator, intake, pivot, wrist);
    }

    public void periodic() {
        robotStates.publishValues();
        robotStates.applyValues();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
