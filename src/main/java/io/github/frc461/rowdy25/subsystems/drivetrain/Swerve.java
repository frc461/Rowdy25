package io.github.frc461.rowdy25.subsystems.drivetrain;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.commands.drive.DirectMoveToPoseCommand;
import io.github.frc461.rowdy25.commands.drive.PathfindToPoseAvoidingReefCommand;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.commands.drive.DriveCommand;
import io.github.frc461.rowdy25.commands.auto.SearchForObjectCommand;
import io.github.frc461.rowdy25.subsystems.localizer.Localizer;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;

import static edu.wpi.first.units.Units.Amps;

/**
 * CTRE Phoenix 6 SwerveDrivetrain subsystem implementation for the Rowdy 25 robot.
 * <p>
 * Extends {@link SwerveDrivetrain} and implements {@link Subsystem} for command-based
 * usage. Manages drive modes (idle, rotating, translating, auto-heading), orchestrates
 * autonomous pathfinding commands to reef branches, coral stations, algae targets,
 * the net, and the processor. Integrates with {@link Localizer} for field-relative
 * localization and provides utility commands for direct pose movement and object tracking.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 * @author Leo Minton, <a href="https://github.com/leo-minton">GitHub</a>
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    /** Drive mode enumeration defining the robot's current driving behavior. */
    public enum DriveMode {
        /** No movement. */
        IDLE,
        /** Rotating at normal speed. */
        ROTATING,
        /** Rotating at fast speed. */
        FAST_ROTATING,
        /** Translating without rotating. */
        TRANSLATING,
        /** Heading toward the nearest reef branch. */
        BRANCH_HEADING,
        /** Heading toward the nearest L1 reef branch. */
        BRANCH_L1_HEADING,
        /** Heading toward the nearest reef AprilTag. */
        REEF_TAG_HEADING,
        /** Heading opposite the nearest reef AprilTag. */
        REEF_TAG_OPPOSITE_HEADING,
        /** Heading toward a detected vision object. */
        OBJECT_HEADING,
        /** Heading toward the coral station. */
        CORAL_STATION_HEADING,
        /** Heading toward the processor. */
        PROCESSOR_HEADING,
        /** Heading toward the net. */
        NET_HEADING
    }

    /** The current drive mode. */
    private DriveMode currentMode;

    /** The localization subsystem for field-relative pose estimation. */
    public final Localizer localizer = new Localizer(this);

    /** Telemetry publisher for swerve drivetrain state. */
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(this);

    /** The CTRE Orchestra instance for playing music through swerve motors. */
    public final Orchestra orchestra = new Orchestra();

    /** List of triggers that detect module stalling (current spike + no motion). */
    private final List<Trigger> moduleStuck = new ArrayList<>();

    /** List of boolean suppliers for detecting motor current stalls. */
    private final List<BooleanSupplier> motorStalling = new ArrayList<>();

    /* Swerve Command Requests */

    /** Field-centric drive request. */
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    /** Robot-centric drive request. */
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    /** Swerve drive brake (x-mode) request. */
    private final SwerveRequest.SwerveDriveBrake xMode = new SwerveRequest.SwerveDriveBrake();

    /** Whether the operator perspective has been applied at least once. */
    private boolean hasAppliedDefaultRotation;

    /** Whether auto-heading is currently active. */
    private boolean autoHeading;

    /** The heading to maintain while translating without rotating. */
    public double consistentHeading;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     */
    public Swerve() {
        /* ah, */ super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                Constants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS,
                Constants.SwerveConstants.FRONT_LEFT,
                Constants.SwerveConstants.FRONT_RIGHT,
                Constants.SwerveConstants.BACK_LEFT,
                Constants.SwerveConstants.BACK_RIGHT
        );

        currentMode = DriveMode.IDLE;

        Song.playRandom(this, Song.startupSongs);

        if (Utils.isSimulation()) {
            new SwerveSim(this).startSimThread();
        }

        AutoBuilder.configure(
                localizer::getStrategyPose,
                localizer::setPoses,
                () -> getKinematics().toChassisSpeeds(getState().ModuleStates),
                (speeds, feedforwards) -> setControl(new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
                ),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P,
                                0,
                                0
                        ),
                        new PIDConstants(
                                Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P,
                                0,
                                0
                        )
                ),
                Constants.AutoConstants.ROBOT_CONFIG,
                () -> Constants.ALLIANCE_SUPPLIER.get() == Alliance.Red,
                this
        );

        Arrays.stream(getModules()).map(SwerveModule::getDriveMotor)
                .forEach(motor -> motorStalling.add(() -> motor.getStatorCurrent().getValueAsDouble() > Constants.SwerveConstants.SLIP_CURRENT.in(Amps)));

        motorStalling.forEach(motorStalling -> {
            moduleStuck.add(
                    new Trigger((motorStalling)).debounce(0.25).and(() -> {
                        ChassisSpeeds speeds = getState().Speeds;
                        double velocityMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
                        double rotVel = speeds.omegaRadiansPerSecond;
                        return velocityMagnitude < 0.1 && rotVel < 0.25;
                    })
            );
        });

        hasAppliedDefaultRotation = false;
        autoHeading = true;
        consistentHeading = 0.0;
    }

    /**
     * Returns the current drive mode.
     *
     * @return The current drive mode.
     */
    public DriveMode getCurrentMode() {
        return currentMode;
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply.
     * @return Command to run.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Creates a field-centric drive command with the given input suppliers.
     *
     * @param elevatorHeight Supplier for the current elevator height.
     * @param straight Supplier for the forward/backward joystick axis.
     * @param strafe Supplier for the left/right joystick axis.
     * @param rotJoystick Supplier for the rotation joystick axis.
     * @param rotLeft Supplier for left rotation button.
     * @param rotRight Supplier for right rotation button.
     * @param fastRotLeft Supplier for fast left rotation button.
     * @param fastRotRight Supplier for fast right rotation button.
     * @return The drive command.
     */
    public Command driveFieldCentric(
            DoubleSupplier elevatorHeight,
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rotJoystick,
            DoubleSupplier rotLeft,
            DoubleSupplier rotRight,
            BooleanSupplier fastRotLeft,
            BooleanSupplier fastRotRight
    ) {
        return new DriveCommand(
                this,
                fieldCentric,
                elevatorHeight,
                straight,
                strafe,
                rotJoystick,
                rotLeft,
                rotRight,
                fastRotLeft,
                fastRotRight,
                () -> currentMode,
                () -> autoHeading
        );
    }

    /**
     * Creates a command that moves directly toward a detected object using vision.
     *
     * @param objectObtained Supplier that returns true when the object has been obtained.
     * @param objectLabelClass The type of object to search for (e.g., coral, algae).
     * @return The object-search command.
     */
    public Command directMoveToObject(BooleanSupplier objectObtained, PhotonUtil.Color.TargetClass objectLabelClass) {
        return new SearchForObjectCommand(this, fieldCentric, objectObtained, objectLabelClass, 2.5);
    }

    /**
     * Creates a command that pushes an alliance partner robot out of the way.
     * Applies a backward robot-centric velocity for 0.5 seconds.
     *
     * @return The partner push command.
     */
    public Command pushAlliancePartnerOut() {
        return applyRequest(() -> robotCentric.withVelocityX(-1.0))
                .withDeadline(Commands.waitSeconds(0.5))
                .andThen(this::forceStop);
    }

    /**
     * Creates a pathfinding command to approach the left coral station and ground-intake coral.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToLeftCoralStationGroundIntakeCoral(RobotStates robotStates) {
        return Commands.defer(
                () -> new InstantCommand(robotStates::toggleGroundCoralState)
                        .andThen(new PathfindToPoseAvoidingReefCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                Constants.FAR_LEFT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER)
                                        .plus(new Transform2d(2.0, 0, Rotation2d.fromDegrees(10)))
                        )).andThen(new WaitUntilCommand(PhotonUtil.Color::hasCoralTargets))
                                .until(PhotonUtil.Color::hasCoralTargets)
                                .andThen(directMoveToObject(robotStates.intake::hasCoral, PhotonUtil.Color.TargetClass.CORAL)),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach the right coral station and ground-intake coral.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToRightCoralStationGroundIntakeCoral(RobotStates robotStates) {
        return Commands.defer(
                () -> new InstantCommand(robotStates::toggleGroundCoralState)
                        .andThen(new PathfindToPoseAvoidingReefCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                Constants.FAR_RIGHT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER)
                                        .plus(new Transform2d(2.0, 0, Rotation2d.fromDegrees(10)))
                        )).andThen(new WaitUntilCommand(PhotonUtil.Color::hasCoralTargets))
                                .until(PhotonUtil.Color::hasCoralTargets)
                                .andThen(directMoveToObject(robotStates.intake::hasCoral, PhotonUtil.Color.TargetClass.CORAL)),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach the left coral station for intake.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToLeftCoralStation(RobotStates robotStates) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        RobotPoses.CoralStation.getRobotPosesAtEachCoralStation().get(0).interpolate(Constants.FAR_LEFT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER), 0.25)
                ).until(() -> isStuck()
                                && localizer.getStrategyPose().getTranslation().getDistance(localizer.nearestRobotPoseAtCoralStation.getTranslation()) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT * 5
                                || robotStates.intake.coralEntered())
                        .alongWith(new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.CORAL_STATION)).andThen(() -> robotStates.toggleCoralStationState(true))),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach the right coral station for intake.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToRightCoralStation(RobotStates robotStates) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        RobotPoses.CoralStation.getRobotPosesAtEachCoralStation().get(1).interpolate(Constants.FAR_RIGHT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER), 0.25)
                ).until(() -> isStuck()
                                && localizer.getStrategyPose().getTranslation().getDistance(localizer.nearestRobotPoseAtCoralStation.getTranslation()) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT * 5
                                || robotStates.intake.coralEntered())
                        .alongWith(new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.CORAL_STATION)).andThen(() -> robotStates.toggleCoralStationState(true))),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach the nearest left reef branch for coral scoring.
     * Transitions from pathfinding to direct drive at the scoring location.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToNearestLeftBranch(RobotStates robotStates) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        localizer.nearestRobotPosesNearBranchPair.getFirst()
                ).until(() -> robotStates.atTransitionStateLocation(RobotStates.State.L4_CORAL)
                        && localizer.sameSideAsTarget(localizer.nearestRobotPosesAtBranchPair.getFirst()))
                        .andThen(new DirectMoveToPoseCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                localizer.nearestRobotPosesAtBranchPair.getFirst(),
                                robotStates.getCurrentAutoLevel() == FieldUtil.Reef.Level.L4 ? 2.5 : Constants.MAX_VEL
                        )).andThen(
                                new WaitUntilCommand(robotStates.atAutoScoreState.and(robotStates::atScoringLocation))
                                        .andThen(robotStates::toggleAutoLevelCoralState)
                                        .onlyIf(() -> autoHeading)
                        ).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.L4_CORAL))
                                        .andThen(() -> robotStates.toggleAutoLevelCoralState(true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach the nearest right reef branch for coral scoring.
     * Transitions from pathfinding to direct drive at the scoring location.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToNearestRightBranch(RobotStates robotStates) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        localizer.nearestRobotPosesNearBranchPair.getSecond()
                ).until(() -> robotStates.atTransitionStateLocation(RobotStates.State.L4_CORAL)
                        && localizer.sameSideAsTarget(localizer.nearestRobotPosesAtBranchPair.getSecond()))
                        .andThen(new DirectMoveToPoseCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                localizer.nearestRobotPosesAtBranchPair.getSecond(),
                                robotStates.getCurrentAutoLevel() == FieldUtil.Reef.Level.L4 ? 2.5 : Constants.MAX_VEL
                        )).andThen(
                                new WaitUntilCommand(robotStates.atAutoScoreState.and(robotStates::atScoringLocation))
                                        .andThen(robotStates::toggleAutoLevelCoralState)
                                        .onlyIf(() -> autoHeading)
                        ).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.L4_CORAL))
                                        .andThen(() -> robotStates.toggleAutoLevelCoralState(true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach a specific pirate branch for coral scoring.
     * Transitions from pathfinding to direct drive at the scoring location.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @param location The reef branch scoring location (left or right side).
     * @param level The reef level (L1-L4).
     * @return The pathfinding command.
     */
    public Command pathFindToScoringLocation(RobotStates robotStates, FieldUtil.Reef.ScoringLocation location, FieldUtil.Reef.Level level) {
        return Commands.defer(
                () -> new InstantCommand(() -> robotStates.setCurrentAutoLevel(level))
                        .andThen(new PathfindToPoseAvoidingReefCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                RobotPoses.Reef.getRobotPoseNearBranch(localizer.currentRobotScoringSetting, location)
                        )).until(() -> robotStates.atTransitionStateLocation(RobotStates.State.L4_CORAL))
                        .andThen(new DirectMoveToPoseCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                RobotPoses.Reef.getRobotPoseAtBranch(localizer.currentRobotScoringSetting, location),
                                robotStates.getCurrentAutoLevel() == FieldUtil.Reef.Level.L4 ? 2.5 : Constants.MAX_VEL
                        )).raceWith(new WaitUntilCommand(new Trigger(() -> robotStates.nearStateLocation(RobotStates.State.L4_CORAL)).debounce(1))).andThen(
                                new WaitUntilCommand(robotStates.atAutoScoreState).withTimeout(0.5)
                                        .andThen(robotStates::toggleAutoLevelCoralState)
                        ).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.L4_CORAL, true))
                                        .andThen(() -> robotStates.toggleAutoLevelCoralState(true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach the nearest algae on the reef for removal.
     * The robot will drive to the algae, wait until stuck (indicating algae is grabbed),
     * then back away and stow.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToNearestAlgaeOnReef(RobotStates robotStates) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        localizer.nearestRobotPoseNearAlgaeReef
                ).until(() -> robotStates.atTransitionStateLocation(RobotStates.State.LOW_REEF_ALGAE) && robotStates.atReefAlgaeState.getAsBoolean())
                        .andThen(
                                new DirectMoveToPoseCommand(
                                        this,
                                        fieldCentric,
                                        robotStates.elevator::getPosition,
                                        localizer.nearestRobotPoseAtAlgaeReef,
                                        Constants.MAX_VEL
                                ).until(robotStates.intake::algaeStuck)
                                        .andThen(new DirectMoveToPoseCommand(
                                                this,
                                                fieldCentric,
                                                robotStates.elevator::getPosition,
                                                localizer.nearestRobotPoseNearAlgaeReef,
                                                Constants.MAX_VEL
                                        ).withTimeout(0.5))
                        ).andThen(robotStates::setStowState).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.HIGH_REEF_ALGAE))
                                        .andThen(() -> robotStates.toggleReefAlgaeState(localizer.nearestAlgaeIsHigh, true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach algae on a specific reef side for removal.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @param side The reef side to approach.
     * @return The pathfinding command.
     */
    public Command pathFindToAlgaeOnReef(RobotStates robotStates, FieldUtil.Reef.Side side) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        RobotPoses.Reef.getRobotPoseNearReef(side)
                ).until(() -> robotStates.atTransitionStateLocation(RobotStates.State.LOW_REEF_ALGAE) && robotStates.atReefAlgaeState.getAsBoolean())
                        .andThen(
                                new DirectMoveToPoseCommand(
                                        this,
                                        fieldCentric,
                                        robotStates.elevator::getPosition,
                                        RobotPoses.Reef.getRobotPoseAtAlgaeReef(side),
                                        Constants.MAX_VEL
                                ).until(robotStates.intake::algaeStuck)
                                        .andThen(new DirectMoveToPoseCommand(
                                                this,
                                                fieldCentric,
                                                robotStates.elevator::getPosition,
                                                RobotPoses.Reef.getRobotPoseNearReef(side),
                                                Constants.MAX_VEL
                                        ).withTimeout(0.5))
                        ).andThen(robotStates::setStowState).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.HIGH_REEF_ALGAE, true))
                                        .andThen(() -> robotStates.toggleReefAlgaeState(FieldUtil.Reef.Side.algaeIsHigh(side), true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach and score at the net.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @param randomized Whether to randomize the net scoring position.
     * @return The pathfinding command.
     */
    public Command pathFindToNet(RobotStates robotStates, boolean randomized) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        (randomized ? localizer.randomizeNetScoringPose() : localizer.centerNetScoringPose()).plus(new Transform2d(
                                Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE,
                                0,
                                Rotation2d.kZero
                        ).inverse())
                ).until(robotStates.atNetState)
                        .andThen(new DirectMoveToPoseCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                localizer.randomizedRobotPoseAtNet,
                                1.0
                        )).andThen(new WaitUntilCommand(new Trigger(() -> robotStates.nearStateLocation(RobotStates.State.NET)).debounce(0.5))).andThen(
                                new WaitUntilCommand(robotStates.atNetState).withTimeout(0.0)
                                        .andThen(robotStates::toggleNetState)
                                        .onlyIf(() -> autoHeading)
                        ).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.NET))
                                        .andThen(() -> robotStates.toggleNetState(true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Creates a pathfinding command to approach and score at the processor.
     *
     * @param robotStates The robot states subsystem for state tracking.
     * @return The pathfinding command.
     */
    public Command pathFindToProcessor(RobotStates robotStates) {
        return Commands.defer(
                () -> new PathfindToPoseAvoidingReefCommand(
                        this,
                        fieldCentric,
                        robotStates.elevator::getPosition,
                        localizer.currentAllianceSideRobotPoseAtProcessor.plus(new Transform2d(
                                Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_TRANSITION,
                                0,
                                Rotation2d.kZero
                        ))
                ).until(() -> robotStates.atTransitionStateLocation(RobotStates.State.NET) && robotStates.atNetState.getAsBoolean())
                        .andThen(new DirectMoveToPoseCommand(
                                this,
                                fieldCentric,
                                robotStates.elevator::getPosition,
                                localizer.currentAllianceSideRobotPoseAtProcessor

                        )).andThen(
                                new WaitUntilCommand(robotStates.atProcessorState.and(robotStates::atScoringLocation))
                                        .andThen(robotStates::toggleProcessorState)
                                        .onlyIf(() -> autoHeading)
                        ).alongWith(
                                new WaitUntilCommand(() -> robotStates.atTransitionStateLocation(RobotStates.State.PROCESSOR))
                                        .andThen(() -> robotStates.toggleProcessorState(true))
                        ),
                Set.of(this)
        );
    }

    /**
     * Checks if any swerve module is currently stuck (high current + no motion).
     *
     * @return True if at least one module is stuck.
     */
    public boolean isStuck() {
        return moduleStuck.stream().map(Trigger::getAsBoolean).toList().contains(true);
    }

    /**
     * Checks if the robot is in a fully teleoperated drive mode (not in an auto.heading mode).
     *
     * @return True if in IDLE, ROTATING, FAST_ROTATING, or TRANSLATING mode.
     */
    public boolean isFullyTeleop() {
        return currentMode == DriveMode.IDLE
                || currentMode == DriveMode.ROTATING
                || currentMode == DriveMode.FAST_ROTATING
                || currentMode == DriveMode.TRANSLATING;
    }

    /**
     * Checks if auto-heading is currently active.
     *
     * @return True if auto-heading is enabled.
     */
    public boolean isAutoHeading() {
        return autoHeading;
    }

    /** Stops all swerve module movement immediately. */
    public void forceStop() {
        setControl(fieldCentric
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
    }

    /** Toggles auto-heading on/off. */
    public void toggleAutoHeading() {
        autoHeading = !autoHeading;
    }

    /** Sets the drive mode to IDLE. */
    public void setIdleMode() {
        currentMode = DriveMode.IDLE;
    }

    /** Sets the drive mode to ROTATING. */
    public void setRotatingMode() {
        currentMode = DriveMode.ROTATING;
    }

    /** Sets the drive mode to FAST_ROTATING. */
    public void setFastRotatingMode() {
        currentMode = DriveMode.FAST_ROTATING;
    }

    /** Sets the drive mode to TRANSLATING. */
    public void setTranslatingMode() {
        currentMode = DriveMode.TRANSLATING;
    }

    /** Sets the drive mode to BRANCH_HEADING. */
    public void setBranchHeadingMode() {
        currentMode = DriveMode.BRANCH_HEADING;
    }

    /** Sets the drive mode to BRANCH_L1_HEADING. */
    public void setBranchHeadingL1Mode() {
        currentMode = DriveMode.BRANCH_L1_HEADING;
    }

    /** Sets the drive mode to REEF_TAG_HEADING. */
    public void setReefTagHeadingMode() {
        currentMode = DriveMode.REEF_TAG_HEADING;
    }

    /** Sets the drive mode to REEF_TAG_OPPOSITE_HEADING. */
    public void setReefTagOppositeHeadingMode() {
        currentMode = DriveMode.REEF_TAG_OPPOSITE_HEADING;
    }

    /** Sets the drive mode to OBJECT_HEADING (for vision target tracking). */
    public void setObjectHeadingMode() {
        currentMode = DriveMode.OBJECT_HEADING;
    }

    /** Sets the drive mode to CORAL_STATION_HEADING. */
    public void setCoralStationHeadingMode() {
        currentMode = DriveMode.CORAL_STATION_HEADING;
    }

    /** Sets the drive mode to PROCESSOR_HEADING. */
    public void setProcessorHeadingMode() {
        currentMode = DriveMode.PROCESSOR_HEADING;
    }

    /** Sets the drive mode to NET_HEADING. */
    public void setNetHeadingMode() {
        currentMode = DriveMode.NET_HEADING;
    }

    /**
     * Periodic subsystem update called by the robot main loop.
     *
     * <p>This method ensures the operator perspective rotation is applied once after
     * startup (or when the Driver Station is disabled), synchronizes heading sources
     * while disabled, manages Orchestra music playback during enable/disable
     * transitions, publishes drivetrain telemetry, and forwards periodic updates
     * to the {@link Localizer} instance.
     *
     * @implNote Keep this lightweight; it runs on the main robot thread.
     */
    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!hasAppliedDefaultRotation || DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                    Constants.ALLIANCE_SUPPLIER.get() == Alliance.Blue
                            ? Constants.BLUE_DEFAULT_ROTATION
                            : Constants.RED_DEFAULT_ROTATION
            );
            hasAppliedDefaultRotation = true;
        }

        if (DriverStation.isDisabled()) {
            localizer.syncRotations();
        }

        if (DriverStation.isDisabled() && !orchestra.isPlaying() && !Song.tenSeconds()) {
            Song.playRandom(this, Song.disableSongs);
        } if (!DriverStation.isDisabled() && orchestra.isPlaying() || Song.tenSeconds()) {
            orchestra.stop();
        }

        swerveTelemetry.publishValues();
        localizer.periodic();
    }
}