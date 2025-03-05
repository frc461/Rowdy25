package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.EquationUtil;
import frc.robot.util.FieldUtil;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class PathfindToPoseWithReefObstacleCommand extends Command { // TODO: IMPLEMENT SPHERICAL MODEL TO AVOID REEF COLLISION
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private final Pose2d targetPose;
    private Pose2d temporaryTargetPose;
    private boolean xPosDone, yPosDone, yawDone, end;

    public PathfindToPoseWithReefObstacleCommand(
            Pose2d targetPose,
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight
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
        temporaryTargetPose = new Pose2d();
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        temporaryTargetPose = new Pose2d();
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        temporaryTargetPose = getTemporaryTargetPose(currentPose);

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(MathUtil.clamp(
                                EquationUtil.linearOutput(temporaryTargetPose.getX() - currentPose.getX(), 1.0),
                                -Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()),
                                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
                        ))
                        .withVelocityY(MathUtil.clamp(
                                EquationUtil.linearOutput(temporaryTargetPose.getY() - currentPose.getY(), 1.0),
                                -Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()),
                                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
                        ))
                        .withRotationalRate(yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                temporaryTargetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble()))
        );

        xPosDone = Math.abs(currentPose.getX() - targetPose.getX())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yPosDone = Math.abs(currentPose.getY() - targetPose.getY())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yawDone = Math.abs(MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180))
                < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;

        if (xPosDone && yPosDone && yawDone) {
            swerve.forceStop();
            swerve.consistentHeading = currentPose.getRotation().getDegrees();
            end = true;
        }
    }

    private Pose2d getTemporaryTargetPose(Pose2d currentPose) {
        Rotation2d angleToReefCenter = FieldUtil.Reef.getAngleFromReefCenter(currentPose);

        return new Pose2d(Translation2d.kZero, angleToReefCenter);
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
