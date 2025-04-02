package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.EquationUtil;

import java.util.function.DoubleSupplier;

public class DirectMoveToPoseCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private final Pose2d targetPose;
    private final double maxVelocity;
    private boolean xPosDone, yPosDone, yawDone, end;

    public DirectMoveToPoseCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose
    ) {
        this(swerve, fieldCentric, elevatorHeight, targetPose, 1.0);
    }

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

    @Override
    public void end(boolean interrupted) {
        swerve.forceStop();
        swerve.consistentHeading = swerve.localizer.getStrategyPose().getRotation().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
