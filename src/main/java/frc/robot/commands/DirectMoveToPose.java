package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.EquationUtil;
import frc.robot.util.FieldUtil;

import java.util.function.DoubleSupplier;

public class DirectMoveToPose extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private final Pose2d targetPose;
    private boolean xPosDone, yPosDone, yawDone, end;

    public DirectMoveToPose(Swerve swerve, SwerveRequest.FieldCentric fieldCentric, DoubleSupplier elevatorHeight, Pose2d targetPose) {
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
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();

        double velocity = MathUtil.clamp(
                Math.max(
                        EquationUtil.expOutput(targetPose.getTranslation().getDistance(currentPose.getTranslation()), 0.02, 50),
                        Math.min(EquationUtil.linearOutput(targetPose.getTranslation().getDistance(currentPose.getTranslation()), 2.0), 2.0) // TODO WAIT (UNTIL SLOW WORKS): OPTIMIZE SPEED
                ),
                -Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()),
                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
        );

        double velocityHeadingRadians = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
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
            swerve.forceStop();
            swerve.consistentHeading = currentPose.getRotation().getDegrees();
            end = true;
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
