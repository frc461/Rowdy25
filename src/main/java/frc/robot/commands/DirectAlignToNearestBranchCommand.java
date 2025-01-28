package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.ExpUtil;
import frc.robot.util.FieldUtil;

public class DirectAlignToNearestBranchCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final Pose2d targetPose;
    private final PIDController yawController;
    private boolean end;

    public DirectAlignToNearestBranchCommand(Swerve swerve, SwerveRequest.FieldCentric fieldCentric) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        targetPose = FieldUtil.Coral.getNearestBranchPose(swerve.localizer.getStrategyPose()).rotateBy(Rotation2d.kPi);
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        end = swerve.localizer.getStrategyPose().getTranslation().getDistance(targetPose.getTranslation())
                > Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO + 0.5;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        double xError = currentPose.getX() - targetPose.getX();
        double yError = currentPose.getY() - targetPose.getY();
        double yawError = currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees();

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(ExpUtil.output(
                                Math.abs(xError), 2.0, 0.8, 10.0
                        ) * (xError < 0 ? 1 : -1))
                        .withVelocityY(ExpUtil.output(
                                Math.abs(yError), 2.0, 0.8, 10.0
                        ) * (yError < 0 ? 1 : -1))
                        .withRotationalRate(yawController.calculate(
                                yawError,
                                0.0
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL)
        );
        if (Math.hypot(xError, yError) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT
                && Math.abs(yawError) < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT) {
            swerve.forceStop();
            end = true;
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
