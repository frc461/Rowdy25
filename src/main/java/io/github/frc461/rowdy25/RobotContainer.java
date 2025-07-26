package io.github.frc461.rowdy25;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.frc461.rowdy25.autos.AutoManager;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.SysID;

/**
 * RobotContainer is a functional framework instantiated upon robot initialization. The instance would contain all functionality for the robot, including subsystems, commands, and control.
 *
 * <p>The RobotContainer class is a fundamental class to initiate functionality, namely the superstructure (integrated mechanism characterization), Xbox controller binds (robot control), telemetry (for verbosity), autonomous control configurations and other automations.
 *
 */
public class RobotContainer {
    /**
     * {@link RobotStates} is a robot characterization class. The instance would initialize and integrate all subsystems into a superstructure and its states and actions/routines.
     */
    /* Superstructure */
    private final RobotStates robotStates = new RobotStates();

    /**
     * {@link AutoManager} is a custom-built autonomous mode configurator for Rowdy25. The instance would initialize the choosers to specify certain aspects of autonomous mode (e.g., which branches to score coral, which algae to grab off the branch and score) and dynamically generate a command containing actions based on the selected option for each chooser.
     */
    /* Auto Chooser & Configurator */
    private final AutoManager autoManager = new AutoManager(robotStates);

    /**
     * {@link SysID} is a utility to configure routines that can be run to determine practical electric outputs for motors to achieve a certain condition. In other words, a SysID routine can be applied to a motor to tune its positional control through SVAG (feedforward) or PID (feedback). TODO add information about positional control
     */
    /* Sys ID */
    private final SysID sysID = new SysID(robotStates.swerve);

    /**
     * Two {@link CommandXboxController}s are defined to bind robot routines for teleoperated control.
     */
    /* Controllers */ /* Link to controls here: https://docs.google.com/presentation/d/1jv_hAW3l4z0Rqvi-3pNRN2IdWurtOwojIJf5hFRO108/edit?usp=sharing */
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController opXbox = new CommandXboxController(1);

    /**
     * Constructor for RobotContainer. Configure superstructure state-based actions (i.e., when the robot enters a state whether automatically or by button press, the superstructure runs a routine). Initialize teleoperated controls. Initialize PathPlanner, local pathfinder (a "warmup command" is executed to facilitate), and PathPlanner {@link NamedCommands}. Configure {@link DogLog} for telemetry.
     */
    public RobotContainer() {
        robotStates.configureToggleStateTriggers();
        robotStates.setDefaultCommands(driverXbox, opXbox);
        configurePathPlannerNamedCommands();
        configureButtonBindings();

        DogLog.setOptions(new DogLogOptions(() -> false, false, true, true, false, 5000, () -> false));
        DogLog.setPdh(new PowerDistribution());
        
        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
        FollowPathCommand.warmupCommand().schedule();
    }

    /**
     * Binds commands to keywords that PathPlanner searches for in PathPlannerUI-configured autonomous routines. As an autonomous routine runs, the linked command is executed when the keyword-associated event marker is triggered.
     */
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

    /**
     * Binds buttons of Xbox controllers to robot actions through {@link edu.wpi.first.wpilibj2.command.button.Trigger}s. Bindings can be seen <a href="https://docs.google.com/presentation/d/1jv_hAW3l4z0Rqvi-3pNRN2IdWurtOwojIJf5hFRO108/edit?usp=sharing">here</a>.
     */
    private void configureButtonBindings() {
        driverXbox.a().onTrue(new InstantCommand(robotStates.swerve::toggleAutoHeading)
                        .andThen(robotStates.swerve.localizer::toggleTrustCameras)
                        .andThen(
                                Commands.runEnd(
                                        () -> driverXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0.5),
                                        () -> driverXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0)
                                ).withTimeout(0.25).onlyIf(robotStates.swerve::isAutoHeading)
                        ));

        driverXbox.b().whileTrue(robotStates.swerve.pathFindToProcessor(robotStates));

        driverXbox.x().whileTrue(robotStates.swerve.pathFindToNet(robotStates, true));

        driverXbox.y().onTrue(new InstantCommand(robotStates::setStowState));

        driverXbox.povUp().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setRotations(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero)));
        driverXbox.povDown().onTrue(new InstantCommand(robotStates.swerve.localizer::syncRotations));
        driverXbox.povLeft().whileTrue(Commands.runEnd(robotStates.pivot::activateCageIntake, robotStates.pivot::stopCageIntake));
        driverXbox.povRight().onTrue(new InstantCommand(robotStates::escalateClimb));

        driverXbox.leftStick().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.CENTER_OF_RIGHT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER))));
        driverXbox.rightStick().onTrue(new InstantCommand(() -> robotStates.swerve.localizer.setPoses(Constants.CENTER_OF_LEFT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER))));

        driverXbox.leftBumper().whileTrue(new ConditionalCommand(
                robotStates.swerve.pathFindToNearestLeftBranch(robotStates),
                robotStates.swerve.pathFindToLeftCoralStation(robotStates),
                robotStates.intake::barelyHasCoral
        ));
        driverXbox.rightBumper().whileTrue(new ConditionalCommand(
                robotStates.swerve.pathFindToNearestRightBranch(robotStates),
                robotStates.swerve.pathFindToRightCoralStation(robotStates),
                robotStates.intake::barelyHasCoral
        ));
        driverXbox.leftBumper().and(driverXbox.rightBumper()).whileTrue(
                robotStates.swerve.pathFindToNearestAlgaeOnReef(robotStates)
                        .unless(robotStates.intake::barelyHasCoral)
        );

        driverXbox.start().onTrue(new InstantCommand(robotStates::setClimbState));

        opXbox.povDown().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L4)));

        opXbox.povRight().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L1)));

        opXbox.povLeft().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L3)));

        opXbox.povUp().onTrue(new InstantCommand(() -> robotStates.setCurrentAutoLevel(FieldUtil.Reef.Level.L2)));

        opXbox.leftTrigger().whileTrue(Commands.runEnd(() -> robotStates.intake.setIntakeState(true), robotStates.intake::setIdleState));
        opXbox.rightTrigger().whileTrue(Commands.runEnd(robotStates.intake::setOuttakeState, robotStates.intake::setIdleState));

        opXbox.leftStick().onTrue(new InstantCommand(robotStates::toggleNetState));
        opXbox.rightStick().onTrue(new InstantCommand(robotStates::toggleProcessorState));

        opXbox.leftBumper().onTrue(new InstantCommand(robotStates::toggleAutoLevelCoralState)); // TODO: IMPLEMENT LIST OF CORAL
        opXbox.rightBumper().onTrue(new InstantCommand(robotStates::setStowState));

        opXbox.a().onTrue(new InstantCommand(robotStates::toggleGroundAlgaeState));

        opXbox.b().onTrue(new InstantCommand(robotStates::toggleHighReefAlgaeState));

        opXbox.x().onTrue(new InstantCommand(robotStates::toggleLowReefAlgaeState));

        opXbox.y().onTrue(new InstantCommand(robotStates::toggleCoralStationState));

        opXbox.back().onTrue(new InstantCommand(robotStates::toggleNetState));

        opXbox.start().onTrue(new InstantCommand(robotStates::toggleProcessorState));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(opXbox);
    }

    /**
     * Forwards the robot states method to update telemetry.
     */
    public void periodic() {
        robotStates.publishValues();
    }

    /**
     * Forwards the autonomous manager method to dynamically generate an autonomous command based on selected options from its choosers.
     *
     * @return The final autonomous command to be returned to {@link Robot} to be run as a routine.
     */
    public Command getAutonomousCommand() {
        return autoManager.getFinalAutoCommand();
    }
}
