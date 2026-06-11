package io.github.frc461.rowdy25.commands.auto;

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

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.drivetrain.Swerve;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;

import java.util.*;
import java.util.function.BooleanSupplier;

/**
 * A specialized PathPlanner path-following command that continuously checks for the presence of algae
 * during execution using vision data from {@link PhotonUtil}. If an algae check marker is reached
 * and no algae are detected, the command will immediately interrupt and finish.
 *
 * <p>Extends the fundamental path-following command used by PathPlanner.</p>
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class FollowPathRequiringAlgaeCommand extends FollowPathCommand {
    /** Standard timer used to track the path trajectory progression. */
    private final Timer timer = new Timer();

    /** The robot's swerve drive subsystem. */
    private final Swerve swerve;

    /** The original un-flipped PathPlanner path. */
    private final PathPlannerPath originalPath;

    /** The robot's physical constraints and configurations for path trajectory generation. */
    private final RobotConfig robotConfig;

    /** Supplier that dictates whether to automatically flip the path for the red alliance. */
    private final BooleanSupplier shouldFlipPath;

    /** Supplier that checks if the color camera pipeline is currently detecting any algae objects. */
    private final BooleanSupplier hasAlgaeTargets = PhotonUtil.Color::hasAlgaeTargets;

    /** An actively managed queue of PathPlanner marker events that trigger algae presence checks. */
    private final List<OneShotTriggerEvent> allInstantEvents = new ArrayList<>();

    /** The active computed path considering current field flip configurations. */
    private PathPlannerPath path;

    /** The active trajectory being followed during this command execution. */
    private PathPlannerTrajectory trajectory;

    /** Whether the command has been internally interrupted due to encountering an empty algae marker event. */
    protected boolean interrupted;

    /**
     * Constructs the algae-requiring path following command.
     *
     * @param path The target {@link PathPlannerPath} to follow.
     * @param setAssumedPosition Whether to reset the robot's localizer to the path's optimal starting pose on initialization.
     * @param swerve The robot's {@link Swerve} drivetrain subsystem.
     */
    public FollowPathRequiringAlgaeCommand(PathPlannerPath path, boolean setAssumedPosition, Swerve swerve) {
        /* ah, */ super(
                path,
                swerve.localizer::getStrategyPose,
                () -> swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates),
                (speeds, feedforwards) -> swerve.setControl(new SwerveRequest.ApplyRobotSpeeds()
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
                () -> Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red,
                swerve
        );

        this.swerve = swerve;
        if (setAssumedPosition) {
            path.getStartingHolonomicPose().ifPresent(this.swerve.localizer::setPoses);
        }
        this.originalPath = path;
        this.robotConfig = Constants.AutoConstants.ROBOT_CONFIG;
        this.shouldFlipPath = () -> Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red;

        this.path = this.originalPath;
        Optional<PathPlannerTrajectory> idealTrajectory = this.path.getIdealTrajectory(this.robotConfig);
        idealTrajectory.ifPresent((traj) -> this.trajectory = traj);

        interrupted = false;
    }

    /**
     * Initializes the command by preparing the trajectory based on the robot's current pose and queuing up any specific algae check marker events encoded in the PathPlanner path.
     */
    @Override
    public void initialize() {
        /* ah, */ super.initialize();

        if (this.shouldFlipPath.getAsBoolean() && !this.originalPath.preventFlipping) {
            this.path = this.originalPath.flipPath();
        } else {
            this.path = this.originalPath;
        }

        Pose2d currentPose = swerve.localizer.getStrategyPose();
        ChassisSpeeds currentSpeeds = swerve.getKinematics().toChassisSpeeds(swerve.getState().ModuleStates);
        double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        if (this.path.getIdealStartingState() != null) {
            boolean idealVelocity = Math.abs(linearVel - this.path.getIdealStartingState().velocityMPS()) <= (double)0.25F;
            boolean idealRotation = !this.robotConfig.isHolonomic || Math.abs(currentPose.getRotation().minus(this.path.getIdealStartingState().rotation()).getDegrees()) <= (double)30.0F;
            if (idealVelocity && idealRotation) {
                this.trajectory = this.path.getIdealTrajectory(this.robotConfig).orElseThrow();
            } else {
                this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.robotConfig);
            }
        } else {
            this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.robotConfig);
        }

        List<Event> allEvents = trajectory.getEvents();
        for (Event event : allEvents) {
            if (event instanceof OneShotTriggerEvent && ((OneShotTriggerEvent) event).getEventName().equals(Constants.AutoConstants.ALGAE_CHECK_MARKER)) {
                allInstantEvents.add((OneShotTriggerEvent) event);
            }
        }

        this.timer.reset();
        this.timer.start();
        interrupted = false;
    }

    /**
     * Executes the path following periodic logic while simultaneously checking queued algae markers.
     * If an algae marker timestamp is crossed and the camera detects no algae target, flags the command as interrupted.
     */
    @Override
    public void execute() {
        /* ah, */ super.execute();

        double currentTime = this.timer.get();
        if (!allInstantEvents.isEmpty()) {
            if (allInstantEvents.get(0).getTimestampSeconds() <= currentTime) {
                allInstantEvents.remove(0);
                if (!hasAlgaeTargets.getAsBoolean()) {
                    interrupted = true;
                }
            }
        }
    }

    /**
     * Ends the command, accounting for internal visual algae-check interruptions.
     *
     * @param interrupted Whether the command was externally interrupted or interrupted due to an algae absence.
     */
    @Override
    public void end(boolean interrupted) {
        /* ah, */ super.end(this.interrupted);
        this.interrupted = false;
    }

    /**
     * Returns true if the computed path trajectory has completed or the algae scanner forced an interruption.
     *
     * @return True if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds()) || interrupted;
    }

    /**
     * Returns a WrapperCommand invoking a callback when this command ends. It passes the internal {@code interrupted}
     * condition properly if the path following was aborted early due to an algae-check miss.
     *
     * <p>This method overrides what was previously defined by the {@link Command} class to switch the order of execution of the boolean consumer and the wrapped command end method. Honestly, not sure why.</p>
     *
     * @param end A {@link BooleanConsumer} receiving and resolving the interrupt state of the ended command.
     * @return A {@link WrapperCommand} with appended end logic.
     */
    @Override
    public WrapperCommand finallyDo(BooleanConsumer end) {
        return new WrapperCommand(this) {
            @Override
            public void end(boolean interrupted) {
                end.accept(FollowPathRequiringAlgaeCommand.this.interrupted || interrupted);
                /* ah, */ super.end(interrupted);
            }
        };
    }
}
