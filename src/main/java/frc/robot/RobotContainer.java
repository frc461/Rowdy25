package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoManager;
import frc.robot.constants.Constants;
import frc.robot.util.SysID;

public class RobotContainer {
    /* Superstructure */
    private final RobotStates robotStates = new RobotStates();

    /* Auto Chooser & Configurator */
    private final AutoManager autoManager = new AutoManager();

    /* Sys ID */
    private final SysID sysID = new SysID(robotStates.swerve);

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

    private final CommandXboxController opXbox = new CommandXboxController(1);
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
        robotStates.configureToggleStateTriggers();
        robotStates.setDefaultCommands(driverXbox, opXbox);
        configurePathPlannerNamedCommands();
        configureButtonBindings();

        // DogLogOptions(BooleanSupplier ntPublish, boolean captureNt, boolean captureDs, boolean logExtras, boolean captureConsole, int logEntryQueueCapacity)
        DogLog.setOptions(new DogLogOptions(() -> false, false, true, true, false, 5000));
        DogLog.setPdh(new PowerDistribution());
        
        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configurePathPlannerNamedCommands() {
        NamedCommands.registerCommand(
                "outtake",
                new InstantCommand(robotStates::toggleAutoLevelCoralState)
        );

        NamedCommands.registerCommand(
                "intake",
                new InstantCommand(robotStates::toggleCoralStationState)
        );
    }

    private void configureButtonBindings() {
        // IMPORTANT: WHEN BINDING DRIVER BUTTONS, TRIGGERS NEED TO BE ON FALSE ESPECIALLY WITH BINDINGS THAT INITIATE DRIVE AUTOMATION UPON HOLD DEBOUNCE
        // SO FIGURE OUT LOGIC CORRECTLY AND CAREFULLY

        driverXbox.a().onTrue(new InstantCommand(robotStates.wrist::setL4CoralState)
                .andThen(new WaitUntilCommand(() -> !driverXbox.a().getAsBoolean()))
                .andThen(robotStates.wrist::setStowState));
        driverXbox.b().onTrue(new InstantCommand(robotStates.pivot::toggleRatchet));
//        driverXbox.a().onTrue(new InstantCommand(robotStates.swerve::toggleAutoHeading));
//        driverXbox.b().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.FAR_RIGHT_CORAL_STATION)));
//        driverXbox.x().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.FAR_LEFT_CORAL_STATION)));
//        driverXbox.y().whileTrue(robotStates.swerve.directMoveToNearestBranch(robotStates.elevator::getPosition));
//
//        driverXbox.leftBumper().onTrue(new InstantCommand(robotStates.swerve::setBranchHeadingMode) // TODO SHOP: TEST THIS
//                .andThen(new WaitUntilCommand(() -> !driverXbox.leftBumper().getAsBoolean()))
//                .andThen(robotStates.swerve::setIdleMode));
//        driverXbox.rightBumper().onTrue(new InstantCommand(robotStates.swerve::setCoralStationHeadingMode)
//                .andThen(new WaitUntilCommand(() -> !driverXbox.rightBumper().getAsBoolean()))
//                .andThen(robotStates.swerve::setIdleMode));
//
//        opXbox.povDown().onTrue(new InstantCommand(robotStates::toggleL4CoralState));
//
//        opXbox.povRight().onTrue(new InstantCommand(robotStates::toggleL1CoralState));
//
//        opXbox.povLeft().onTrue(new InstantCommand(robotStates::toggleL3CoralState));
//
//        opXbox.povUp().onTrue(new InstantCommand(robotStates::toggleL2CoralState));

//        opXbox.povDown().onTrue(new ConditionalCommand(
//                new ConditionalCommand(
//                        new InstantCommand(robotStates::toggleL4CoralState),
//                        new InstantCommand(robotStates.pivot::toggleRatchet),
//                        () -> !robotStates.intake.hasCoral()
//                ),
//                new InstantCommand(robotStates::toggleGroundCoralState),
//                () -> overrideNonessentialOpControls
//        ));
//
//        opXbox.povRight().onTrue(new ConditionalCommand(
//                new ConditionalCommand(
//                        new InstantCommand(robotStates::toggleL1CoralState),
//                        new InstantCommand(robotStates::toggleHighReefAlgaeState),
//                        robotStates.intake::hasCoral
//                ),
//                new InstantCommand(robotStates::toggleProcessorState),
//                () -> overrideNonessentialOpControls
//        ));
//
//        opXbox.povLeft().onTrue(new ConditionalCommand(
//                new ConditionalCommand(
//                        new InstantCommand(robotStates::toggleL3CoralState),
//                        new InstantCommand(robotStates::toggleLowReefAlgaeState),
//                        robotStates.intake::hasCoral
//                ),
//                new InstantCommand(robotStates::toggleNetState),
//                () -> overrideNonessentialOpControls
//        ));
//
//        opXbox.povUp().onTrue(new ConditionalCommand(
//                new ConditionalCommand(
//                        new InstantCommand(robotStates::toggleL2CoralState),
//                        new InstantCommand(robotStates::toggleCoralStationState),
//                        robotStates.intake::hasCoral
//                ),
//                new InstantCommand(robotStates::toggleGroundAlgaeState),
//                () -> overrideNonessentialOpControls
//        ));

//        opXbox.leftTrigger().onTrue(new InstantCommand(() -> robotStates.intake.setIntakeState(true))
//                .andThen(new WaitUntilCommand(() -> !opXbox.leftTrigger().getAsBoolean()))
//                .andThen(robotStates.intake::setIdleState));
//        opXbox.rightTrigger().onTrue(new InstantCommand(robotStates.intake::setOuttakeState)
//                .andThen(new WaitUntilCommand(() -> !opXbox.rightTrigger().getAsBoolean()))
//                .andThen(robotStates.intake::setIdleState));
//
//        opXbox.leftBumper().onTrue(new InstantCommand(() -> overrideNonessentialOpControls = !overrideNonessentialOpControls));
//        opXbox.rightBumper().onTrue(new InstantCommand(robotStates::setStowState));
//
//        opXbox.a().onTrue(new InstantCommand(robotStates.pivot::toggleRatchet));
//
//        opXbox.b().onTrue(new InstantCommand(robotStates::toggleHighReefAlgaeState));
//
//        opXbox.x().onTrue(new InstantCommand(robotStates::toggleLowReefAlgaeState));
//
//        opXbox.y().onTrue(new InstantCommand(robotStates::toggleCoralStationState));

//        opXbox.a().onTrue(new ConditionalCommand(
//                new InstantCommand(robotStates::toggleL4CoralState),
//                new InstantCommand(robotStates.pivot::toggleRatchet),
//                robotStates.intake::hasCoral
//        ));
//
//        opXbox.b().onTrue(new ConditionalCommand(
//                new InstantCommand(robotStates::toggleL1CoralState),
//                new InstantCommand(robotStates::toggleHighReefAlgaeState),
//                robotStates.intake::hasCoral
//        ));
//
//        opXbox.x().onTrue(new ConditionalCommand(
//                new InstantCommand(robotStates::toggleL3CoralState),
//                new InstantCommand(robotStates::toggleLowReefAlgaeState),
//                robotStates.intake::hasCoral
//        ));
//
//        opXbox.y().onTrue(new ConditionalCommand(
//                new InstantCommand(robotStates::toggleL2CoralState),
//                new InstantCommand(robotStates::toggleCoralStationState),
//                robotStates.intake::hasCoral
//        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(opXbox);
    }

    public void periodic() {
        robotStates.publishValues();
    }

    public Command getAutonomousCommand() {
        return autoManager.getFinalAutoCommand(robotStates);
    }
}
