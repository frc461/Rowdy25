package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.function.Supplier;

public class PathfindingUntilCloseCommand extends PathfindingCommand {
    private final Pose2d targetPose;
    private final Swerve swerve;
    private final double distance;
    private final boolean continueWhenNear;
    private boolean end;

    public PathfindingUntilCloseCommand(Pose2d pose, double distance, boolean continueWhenNear, Swerve swerve) {
        super(
                pose,
                Constants.AutoConstants.PATH_CONSTRAINTS,
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
                swerve
        );

        targetPose = pose;
        this.swerve = swerve;
        this.distance = distance;
        this.continueWhenNear = continueWhenNear;
        this.end = false;
    }

    @Override
    public void initialize() {
        super.initialize();

        end = swerve.localizer.getStrategyPose().getTranslation().getDistance(targetPose.getTranslation()) < distance;
    }

    @Override
    public void execute() {
        super.execute();

        end = swerve.localizer.getStrategyPose().getTranslation().getDistance(targetPose.getTranslation()) < distance;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (continueWhenNear) {
            swerve.driveDirectToNearestBranch().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || end;
    }
}
