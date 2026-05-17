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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.drivetrain.Swerve;
import io.github.frc461.rowdy25.util.EquationUtil;

import java.util.function.DoubleSupplier;

/**
 * A command that directly moves the robot to a specified pose using field-centric swerve control.
 *
 * <p>
 * This command uses PID control for rotational alignment and calculates velocity based on
 * distance to target pose, with safety limits based on elevator height.
 * </p>
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class DirectMoveToPoseCommand extends Command {
    /** The swerve drivetrain subsystem. */
    private final Swerve swerve;

    /** The field-centric swerve drive request. */
    private final SwerveRequest.FieldCentric fieldCentric;

    /** PID controller responsible for orienting the robot towards the target pose. */
    private final PIDController yawController;

    /** Supplier for the current elevator height. */
    private final DoubleSupplier elevatorHeight;

    /** The target pose to move to. */
    private final Pose2d targetPose;

    /** The maximum allowed velocity. */
    private final double maxVelocity;

    /** True if the robot's X translational error is within an acceptable threshold. */
    private boolean xPosDone;

    /** True if the robot's Y translational error is within an acceptable threshold. */
    private boolean yPosDone;

    /** True if the robot's angular error relative to the target is within an acceptable threshold. */
    private boolean yawDone;

    /** Flag indicating that the command should terminate. */
    private boolean end;

    /**
     * Constructs a DirectMoveToPoseCommand with default maximum velocity.
     *
     * @param swerve The swerve drivetrain subsystem.
     * @param fieldCentric The field-centric drive request configuration.
     * @param elevatorHeight Supplier for current elevator height.
     * @param targetPose The target pose to move to.
     */
    public DirectMoveToPoseCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose
    ) {
        this(swerve, fieldCentric, elevatorHeight, targetPose, 1.0);
    }

    /**
     * Constructs a DirectMoveToPoseCommand with specified maximum velocity.
     *
     * @param swerve The swerve drivetrain subsystem.
     * @param fieldCentric The field-centric drive request configuration.
     * @param elevatorHeight Supplier for current elevator height.
     * @param targetPose The target pose to move to.
     * @param maxVelocity The maximum allowed velocity.
     */
    public DirectMoveToPoseCommand(
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
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    /**
     * Initializes the command by resetting completion flags.
     */
    @Override
    public void initialize() {
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    /**
     * Executes the command's control logic regularly.
     *
     * <p>
     * Calculates required velocity and heading, applies swerve control,
     * and checks position and orientation tolerances for completion.
     * </p>
     */
    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        swerve.localizer.setCurrentTemporaryTargetPose(targetPose);
        double safeMaxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()));

        double velocity = Math.max(
                EquationUtil.expOutput(
                        targetPose.getTranslation().getDistance(currentPose.getTranslation()),
                        2,
                        2 / 7.0,
                        15 / 2.0
                ),
                Math.min(EquationUtil.linearOutput(targetPose.getTranslation().getDistance(currentPose.getTranslation()), 10, -10), safeMaxVelocity)
        );

        double velocityHeadingRadians = targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().getRadians();

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withDeadband(0.0)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(Math.cos(velocityHeadingRadians) * velocity)
                        .withVelocityY(Math.sin(velocityHeadingRadians) * velocity)
                        .withRotationalRate(yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                targetPose.getRotation().getDegrees()
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