package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private double xVel, yVel, rotVel, xOrigVel, yOrigVel, rotOrigVel, transitionPoll;
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
        xVel = 0;
        yVel = 0;
        rotVel = 0;
        xOrigVel = 0;
        yOrigVel = 0;
        rotOrigVel = 0;
        transitionPoll = 0;
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        ChassisSpeeds chassisSpeeds = swerve.getState().Speeds;
        Translation2d fieldRelativeTranslation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(swerve.localizer.getStrategyPose().getRotation().unaryMinus());
        xVel = fieldRelativeTranslation.getX();
        yVel = fieldRelativeTranslation.getY();
        rotVel = chassisSpeeds.omegaRadiansPerSecond;
        xOrigVel = fieldRelativeTranslation.getX();
        yOrigVel = fieldRelativeTranslation.getY();
        rotOrigVel = chassisSpeeds.omegaRadiansPerSecond;
        transitionPoll = 0;
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

        double velocity = MathUtil.clamp(
                EquationUtil.expOutput(
                        targetPose.getTranslation().getDistance(currentPose.getTranslation()),
                        safeMaxVelocity,
                        1.0 / 10.0 * Math.pow(safeMaxVelocity, 0.5) * Math.log(safeMaxVelocity * Math.exp(2.5) + safeMaxVelocity - 1), // TODO SHOP: TEST THESE VALUES
                        10.0 / Math.pow(safeMaxVelocity, 0.7)
                ),
                0,
                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
        );

        double velocityHeadingRadians = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());

        updateVelocities(
                Math.cos(velocityHeadingRadians) * velocity,
                Math.sin(velocityHeadingRadians) * velocity,
                yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                targetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble())
        );

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withDeadband(0.0)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(xVel)
                        .withVelocityY(yVel)
                        .withRotationalRate(rotVel)
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

    private void updateVelocities(double xTargetVel, double yTargetVel, double rotTargetVel) {
        transitionPoll++;
        xVel = Math.pow(0.9, transitionPoll) * xOrigVel + (1 - Math.pow(0.9, transitionPoll)) * xTargetVel;
        yVel = Math.pow(0.9, transitionPoll) * yOrigVel + (1 - Math.pow(0.9, transitionPoll)) * yTargetVel;
        rotVel = Math.pow(0.9, transitionPoll) * rotOrigVel + (1 - Math.pow(0.9, transitionPoll)) * rotTargetVel;
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
