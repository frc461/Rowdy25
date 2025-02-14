package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.FieldUtil;

import java.util.function.DoubleSupplier;

public class DirectMoveToNearestBranchCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController translationController;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private Pose2d targetPose;
    private boolean end;

    public DirectMoveToNearestBranchCommand(Swerve swerve, SwerveRequest.FieldCentric fieldCentric, DoubleSupplier elevatorHeight) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        translationController = new PIDController(
                Constants.SwerveConstants.TRANSLATION_ALIGNMENT_CONTROLLER_P,
                0,
                Constants.SwerveConstants.TRANSLATION_ALIGNMENT_CONTROLLER_D
        );

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        this.elevatorHeight = elevatorHeight;

        targetPose = new Pose2d();
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        Pose2d nearestBranchPose = FieldUtil.Reef.getNearestBranchPose(swerve.localizer.getStrategyPose());
        targetPose = new Pose2d(
                nearestBranchPose.getTranslation(),
                nearestBranchPose.getRotation().rotateBy(Rotation2d.kPi)
        );

        end = swerve.localizer.getStrategyPose().getTranslation().getDistance(targetPose.getTranslation())
                > Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO + 0.5;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        double xError = currentPose.getX() - targetPose.getX();
        double yError = currentPose.getY() - targetPose.getY();
        double yawError = MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180);

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(Math.min(
                                2.0,
                                translationController.calculate(xError, 0) * Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
                        ))
                        .withVelocityY(Math.min(
                                2.0,
                                translationController.calculate(yError, 0) * Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
                        ))
                        .withRotationalRate(yawController.calculate(
                                yawError,
                                0.0
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL)
        );
        if (Math.hypot(xError, yError) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT
                && Math.abs(yawError) < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT) {
            swerve.forceStop();
            swerve.consistentHeading = currentPose.getRotation().getDegrees();
            System.out.println(swerve.consistentHeading);
            end = true;
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
