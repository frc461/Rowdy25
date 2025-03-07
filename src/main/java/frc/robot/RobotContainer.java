package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.AutoManager;
import frc.robot.constants.Constants;
import frc.robot.util.SysID;

import java.util.concurrent.locks.Condition;

public class RobotContainer {
    /* Superstructure */
    private final RobotStates robotStates = new RobotStates();

    /* Auto Chooser & Configurator */
    private final AutoManager autoManager = new AutoManager(robotStates);

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
     * Y: Pathfind to nearest coral station
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

    private boolean overrideNonessentialOpControls = false;

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
                Constants.AutoConstants.OUTTAKE_MARKER,
                new InstantCommand(robotStates::toggleAutoLevelCoralState)
        );

        NamedCommands.registerCommand(
                Constants.AutoConstants.INTAKE_MARKER,
                new InstantCommand(robotStates::toggleCoralStationState)
        );
    }

    private void configureButtonBindings() {
        driverXbox.a().onTrue(new InstantCommand(robotStates.swerve::toggleAutoHeading)
                        .andThen(
                                new InstantCommand(() -> driverXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0.5))
                                        .andThen(new WaitCommand(0.25))
                                        .andThen(() -> driverXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0))
                                        .onlyIf(robotStates.swerve::isAutoHeading)
                        ));
        driverXbox.x().whileTrue(robotStates.swerve.pathFindToNet(robotStates.elevator::getPosition));
        driverXbox.y().whileTrue(robotStates.swerve.pathFindToProcessor(robotStates.elevator::getPosition));

        driverXbox.povUp().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setRotations(Rotation2d.kZero)));
        driverXbox.povDown().onTrue(new InstantCommand(robotStates.swerve.localizer::syncRotations));
        driverXbox.povLeft().onTrue(new InstantCommand(() -> robotStates.climb.manualClimb(-0.9)));
        driverXbox.povLeft().onFalse(new InstantCommand(() -> robotStates.climb.stopClimb(true)));
        driverXbox.povRight().onTrue(new InstantCommand(() -> robotStates.climb.manualClimb(0.9)));
        driverXbox.povRight().onFalse(new InstantCommand(() -> robotStates.climb.stopClimb(false)));

        driverXbox.leftStick().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.CENTER_OF_RIGHT_CORAL_STATION)));
        driverXbox.rightStick().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.CENTER_OF_LEFT_CORAL_STATION)));

        driverXbox.leftBumper().whileTrue(new ConditionalCommand(
                robotStates.swerve.pathFindToNearestLeftBranch(robotStates.elevator::getPosition),
                new ConditionalCommand(
                        robotStates.swerve.pathFindToNet(robotStates.elevator::getPosition),
                        robotStates.swerve.pathFindToLeftCoralStation(robotStates.elevator::getPosition),
                        robotStates.intake::hasAlgae
                ),
                robotStates.intake::hasCoral
        ));
        driverXbox.rightBumper().whileTrue(new ConditionalCommand(
                robotStates.swerve.pathFindToNearestRightBranch(robotStates.elevator::getPosition),
                new ConditionalCommand(
                        robotStates.swerve.pathFindToProcessor(robotStates.elevator::getPosition),
                        robotStates.swerve.pathFindToRightCoralStation(robotStates.elevator::getPosition),
                        robotStates.intake::hasAlgae
                ),
                robotStates.intake::hasCoral
        ));

        opXbox.povDown().onTrue(new InstantCommand(robotStates::toggleL4CoralState));

        opXbox.povRight().onTrue(new InstantCommand(robotStates::toggleL1CoralState));

        opXbox.povLeft().onTrue(new InstantCommand(robotStates::toggleL3CoralState));

        opXbox.povUp().onTrue(new InstantCommand(robotStates::toggleL2CoralState));

        opXbox.leftTrigger().onTrue(new InstantCommand(() -> robotStates.intake.setIntakeState(true)));
        opXbox.leftTrigger().onFalse(new InstantCommand(robotStates.intake::setIdleState));
        opXbox.rightTrigger().onTrue(new InstantCommand(robotStates.intake::setOuttakeState));
        opXbox.rightTrigger().onFalse(new InstantCommand(robotStates.intake::setIdleState));

        opXbox.leftBumper().onTrue(new InstantCommand(() -> overrideNonessentialOpControls = !overrideNonessentialOpControls));
        opXbox.rightBumper().onTrue(new InstantCommand(robotStates::setStowState));

        opXbox.a().onTrue(new InstantCommand(robotStates::escalateClimb));

        opXbox.b().onTrue(new InstantCommand(robotStates::toggleHighReefAlgaeState));

        opXbox.x().onTrue(new InstantCommand(robotStates::toggleLowReefAlgaeState));

        opXbox.y().onTrue(new InstantCommand(robotStates::toggleCoralStationState));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(opXbox);
    }

    public void periodic() {
        robotStates.publishValues();
    }

    public Command getAutonomousCommand() {
        return autoManager.getFinalAutoCommand();
    }
}
