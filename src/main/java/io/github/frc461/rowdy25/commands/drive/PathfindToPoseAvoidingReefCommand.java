package io.github.frc461.rowdy25.commands.drive;

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

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.subsystems.drivetrain.Swerve;
import io.github.frc461.rowdy25.util.EquationUtil;
import io.github.frc461.rowdy25.util.FieldUtil;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;

/**
 * A command that pathfinds to a target pose while dynamically avoiding the reef structure.
 *
 * <p>
 * This command uses a temporary target pose that smoothly interpolates around the reef
 * when the straight-line path would intersect it, using tangent points and reef center
 * calculations to generate obstacle-avoiding waypoints.
 * </p>
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class PathfindToPoseAvoidingReefCommand extends Command {
    /** The swerve drivetrain subsystem. */
    private final Swerve swerve;

    /** The field-centric swerve drive request. */
    private final SwerveRequest.FieldCentric fieldCentric;

    /** PID controller responsible for orienting the robot towards the target pose. */
    private final PIDController yawController;

    /** Supplier for current elevator height. */
    private final DoubleSupplier elevatorHeight;

    /** The target pose to pathfind to. */
    private final Pose2d targetPose;

    /** The maximum allowed velocity. */
    private final double maxVelocity;

    /** The smoothly interpolated temporary target pose used to avoid the reef. */
    private Pose2d smoothTemporaryTargetPose;

    /** True if the robot's X translational error is within an acceptable threshold. */
    private boolean xPosDone;

    /** True if the robot's Y translational error is within an acceptable threshold. */
    private boolean yPosDone;

    /** True if the robot's angular error relative to the target is within an acceptable threshold. */
    private boolean yawDone;

    /** Flag indicating that the command should terminate. */
    private boolean end;

    /**
     * Constructs a PathfindToPoseAvoidingReefCommand with default maximum velocity.
     *
     * @param swerve The swerve drivetrain subsystem.
     * @param fieldCentric The field-centric drive request configuration.
     * @param elevatorHeight Supplier for current elevator height.
     * @param targetPose The target pose to pathfind to.
     */
    public PathfindToPoseAvoidingReefCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose
    ) {
        this(swerve, fieldCentric, elevatorHeight, targetPose, Constants.MAX_VEL);
    }

    /**
     * Constructs a PathfindToPoseAvoidingReefCommand with specified maximum velocity.
     *
     * @param swerve The swerve drivetrain subsystem.
     * @param fieldCentric The field-centric drive request configuration.
     * @param elevatorHeight Supplier for current elevator height.
     * @param targetPose The target pose to pathfind to.
     * @param maxVelocity The maximum allowed velocity.
     */
    public PathfindToPoseAvoidingReefCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose,
            double maxVelocity
    ) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        this.elevatorHeight = elevatorHeight;

        this.targetPose = targetPose;
        this.maxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_VEL);

        smoothTemporaryTargetPose = null;
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    /**
     * Initializes the command, computing the initial temporary target pose and resetting completion flags.
     */
    @Override
    public void initialize() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();

        updateSmoothTargetPose(getTemporaryTargetPose(currentPose));

        smoothTemporaryTargetPose = null;
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    /**
     * Executes the command's control logic every 20ms.
     *
     * <p>
     * Updates the smooth temporary target pose to avoid the reef, calculates required
     * velocity and heading, applies swerve control, and checks position and orientation
     * tolerances for completion.
     * </p>
     */
    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        updateSmoothTargetPose(getTemporaryTargetPose(currentPose));
        swerve.localizer.setCurrentTemporaryTargetPose(smoothTemporaryTargetPose);
        double safeMaxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()));

        double velocity = Math.max(
                EquationUtil.expOutput(
                        smoothTemporaryTargetPose.getTranslation().getDistance(currentPose.getTranslation()),
                        2,
                        2 / 7.0,
                        15 / 2.0
                ),
                Math.min(EquationUtil.linearOutput(smoothTemporaryTargetPose.getTranslation().getDistance(currentPose.getTranslation()), 10, -10), safeMaxVelocity)
        );

        double velocityHeadingRadians = smoothTemporaryTargetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().getRadians();

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withDeadband(0.0)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(Math.cos(velocityHeadingRadians) * velocity)
                        .withVelocityY(Math.sin(velocityHeadingRadians) * velocity)
                        .withRotationalRate(yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                smoothTemporaryTargetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble()))
        );

        xPosDone = Math.abs(currentPose.getX() - targetPose.getX())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yPosDone = Math.abs(currentPose.getY() - targetPose.getY())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yawDone = Math.abs(MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180))
                < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;

        if (xPosDone && yPosDone && yawDone) {
            end = true;
        }
    }

    /**
     * Updates the smooth temporary target pose by interpolating towards the raw temporary pose.
     *
     * <p>
     * If the distance between the current smooth pose and the raw pose exceeds a threshold,
     * the smooth pose is moved incrementally towards the raw pose to prevent abrupt directional changes.
     * </p>
     *
     * @param temporaryPose The raw temporary target pose to interpolate towards.
     */
    private void updateSmoothTargetPose(Pose2d temporaryPose) {
        if (smoothTemporaryTargetPose == null) {
            smoothTemporaryTargetPose = temporaryPose;
        }

        if (smoothTemporaryTargetPose.getTranslation().getDistance(temporaryPose.getTranslation()) > 0.11) {
            Rotation2d headingToTemporaryPose = temporaryPose.getTranslation().minus(smoothTemporaryTargetPose.getTranslation()).getAngle();
            smoothTemporaryTargetPose = new Pose2d(
                    new Pose2d(smoothTemporaryTargetPose.getTranslation(), headingToTemporaryPose)
                            .plus(new Transform2d(0.11, 0, Rotation2d.kZero))
                            .getTranslation(),
                    smoothTemporaryTargetPose.getRotation().interpolate(temporaryPose.getRotation(), 0.25)
            );
        } else {
            smoothTemporaryTargetPose = temporaryPose;
        }
    }

    /**
     * Computes a temporary target pose that avoids driving through the reef.
     *
     * <p>
     * If the robot and target pose are on the same side of the reef, the target pose is used directly.
     * If the robot is close to the reef, it backs out to a safe distance before navigating around.
     * Otherwise, a tangent point around the reef is calculated to create an avoidance waypoint.
     * </p>
     *
     * @param currentPose The robot's current pose.
     * @return A temporary target pose that safely avoids the reef.
     */
    private Pose2d getTemporaryTargetPose(Pose2d currentPose) {
        Translation2d nearestReefCenter = FieldUtil.Reef.getNearestReefCenter(currentPose.getTranslation());
        Rotation2d reefCenterAngleToRobot = FieldUtil.Reef.getAngleFromNearestReefCenter(currentPose);

        if (RobotPoses.Reef.sameSide(currentPose, targetPose)) {
            return targetPose;
        } else if (currentPose.getTranslation().getDistance(nearestReefCenter) < FieldUtil.Reef.REEF_APOTHEM + Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 1.3) {
            Translation2d targetTranslation = new Pose2d(nearestReefCenter, FieldUtil.Reef.getNearestReefTagPose(currentPose, true).getRotation())
                    .plus(new Transform2d(2.0, 0, Rotation2d.kZero))
                    .getTranslation();
            return new Pose2d(targetTranslation, currentPose.getRotation());
        } else {
            Rotation2d reefCenterAngleToTargetPose = targetPose.getTranslation().minus(nearestReefCenter).getAngle();
            Rotation2d temporaryTangentAngle =
                    reefCenterAngleToRobot.rotateBy(Rotation2d.fromDegrees(Math.copySign(
                            90.0,
                            reefCenterAngleToTargetPose.minus(reefCenterAngleToRobot).getDegrees()
                    )));
            return new Pose2d(
                    new Pose2d(
                            new Pose2d(nearestReefCenter, reefCenterAngleToRobot)
                                    .plus(new Transform2d(
                                            2.0,
                                            0,
                                            Rotation2d.kZero)
                                    ).getTranslation(),
                            temporaryTangentAngle
                    ).plus(new Transform2d(
                            1.5,
                            0,
                            Rotation2d.kZero
                    )).getTranslation(),
                    currentPose.getRotation().interpolate(targetPose.getRotation(), 0.25)
            );
        }
    }

    /**
     * Ends the command, safely stopping all module motion and updating the active heading.
     *
     * @param interrupted Whether the command was externally interrupted or canceled early.
     */
    @Override
    public void end(boolean interrupted) {
        swerve.forceStop();
        swerve.consistentHeading = swerve.localizer.getStrategyPose().getRotation().getDegrees();
    }

    /**
     * Checks if the command has finished moving to the target pose within tolerances.
     *
     * @return True if all positional and rotational tolerances are met, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return end;
    }
}