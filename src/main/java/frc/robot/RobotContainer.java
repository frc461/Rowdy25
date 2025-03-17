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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoManager;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import frc.robot.util.SysID;

public class RobotContainer {
    /* Superstructure */
    private final RobotStates robotStates = new RobotStates();

    /* Auto Chooser & Configurator */
    private final AutoManager autoManager = new AutoManager(robotStates);

    /* Sys ID */
    private final SysID sysID = new SysID(robotStates.swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Driver:
     * POV buttons / D-pad:
     * Up: Click - Zero gyro
     * Down: Click - Sync gyro (with pose estimation)
     * Left: Hold - manual lower climb
     * Right: Hold - manual higher climb
     *
     * Triggers:
     * Left: Rotate CCW (double click - FAST)
     * Right: Rotate CW (double click - FAST)
     *
     * Joysticks:
     * Left: Translation
     * Right:
     * Left Button: Reset position to coral left side
     * Right Button: Reset position to coral right side
     *
     * Bumpers:
     * Left: Hold - pathfind to left branch of nearest reef side, net, left coral station, if it has coral, algae, or neither, respectively, and automatically set the robot state when near
     *      Release: Score if applicable
     * Right: Hold - pathfind to right branch of nearest reef side, processor, right coral station, if it has coral, algae, or neither, respectively, and automatically set the robot state when near
     *      Release: Score if applicable
     * Left + Right: Hold - pathfind to nearest side of reef to intake algae if robot doesn't have algae
     *
     * Buttons:
     *
     * A: Click - toggle auto align
     *
     * B:
     *
     * X:
     *
     * Y:
     */

    private final CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / D-pad:
     * Up: Click - L4 auto preset, Double Click - toggle L4 preset
     * Down: Click - L1 auto preset, Double Click - toggle L1 preset
     * Left: Click - L2 auto preset, Double Click - toggle L2 preset
     * Right: Click - L3 auto preset, Double Click - toggle L3 preset
     *
     * Triggers:
     * Left: Hold - intake
     * Right: Hold - outtake
     *
     * Joysticks:
     * Left: Move elevator (x), rotate pivot (y)
     * Right: Rotate wrist
     * Left Button: Click - Net algae score state, Click again - Outtake, stow
     * Right Button: Click - Processor algae score state, Click again - Outtake, stow
     *
     * Bumpers:
     * Left: Click - ???
     * Right: Click - Stow
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

        driverXbox.povUp().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setRotations(Rotation2d.kZero)));
        driverXbox.povDown().onTrue(new InstantCommand(robotStates.swerve.localizer::syncRotations));
        driverXbox.povLeft().onTrue(new InstantCommand(() -> robotStates.climb.move(-0.9)));
        driverXbox.povLeft().onFalse(new InstantCommand(() -> robotStates.climb.stop(true)));
        driverXbox.povRight().onTrue(new InstantCommand(() -> robotStates.climb.move(0.9)));
        driverXbox.povRight().onFalse(new InstantCommand(() -> robotStates.climb.stop(false)));

        new Trigger(() -> Math.hypot(driverXbox.getLeftX(), driverXbox.getLeftY()) > 0.75).onTrue(
                new InstantCommand(() -> driverXbox.setRumble(GenericHID.RumbleType.kBothRumble, 1.0))
                        .andThen(new WaitUntilCommand(() -> Math.hypot(driverXbox.getLeftX(), driverXbox.getLeftY()) < 0.75))
                        .andThen(() -> driverXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0))
        );
        driverXbox.leftStick().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.CENTER_OF_RIGHT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER))));
        driverXbox.rightStick().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.CENTER_OF_LEFT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER))));

        // TODO SHOP: TEST ALL THIS WITH ONLY SWERVE AUTO-SCORING (NOT ROBOTSTATES)
        driverXbox.leftBumper().whileTrue(new ConditionalCommand(
                robotStates.swerve.pathFindToNearestLeftBranch(robotStates)
                        .unless(robotStates.l1CoralState),
                new ConditionalCommand(
                        robotStates.swerve.pathFindToNet(robotStates),
                        robotStates.swerve.pathFindToLeftCoralStation(robotStates),
                        robotStates.intake::hasAlgae
                ),
                robotStates.intake::hasCoral
        ));
        driverXbox.rightBumper().whileTrue(new ConditionalCommand(
                robotStates.swerve.pathFindToNearestRightBranch(robotStates)
                        .unless(robotStates.l1CoralState),
                new ConditionalCommand(
                        robotStates.swerve.pathFindToProcessor(robotStates),
                        robotStates.swerve.pathFindToRightCoralStation(robotStates),
                        robotStates.intake::hasAlgae
                ),
                robotStates.intake::hasCoral
        ));
        driverXbox.leftBumper().and(driverXbox.rightBumper()).whileTrue(
                robotStates.swerve.pathFindToNearestAlgaeOnReef(robotStates)
                        .unless(() -> robotStates.intake.hasAlgae() || robotStates.intake.hasCoral())
        );

        opXbox.povDown().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L4)));

        opXbox.povRight().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L1)));

        opXbox.povLeft().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L3)));

        opXbox.povUp().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L2)));

        opXbox.leftTrigger().onTrue(new InstantCommand(() -> robotStates.intake.setIntakeState(true)));
        opXbox.leftTrigger().onFalse(new InstantCommand(robotStates.intake::setIdleState));
        opXbox.rightTrigger().onTrue(new InstantCommand(robotStates.intake::setOuttakeState));
        opXbox.rightTrigger().onFalse(new InstantCommand(robotStates.intake::setIdleState));

        opXbox.leftStick().onTrue(new InstantCommand(robotStates::toggleNetState));
        opXbox.rightStick().onTrue(new InstantCommand(robotStates::toggleProcessorState));

        opXbox.leftBumper().onTrue(new InstantCommand(robotStates::toggleAutoLevelCoralState)); // TODO: IMPLEMENT LIST OF CORAL
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
