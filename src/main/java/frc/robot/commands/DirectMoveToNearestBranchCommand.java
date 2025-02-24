package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.ExpUtil;
import frc.robot.util.FieldUtil;

import java.util.function.DoubleSupplier;

public class DirectMoveToNearestBranchCommand extends Command { // TODO: IMPLEMENT SPHERICAL MODEL TO AVOID REEF COLLISION
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private Pose2d targetPose;
    private boolean xPosDone, yPosDone, yawDone, end;

    public DirectMoveToNearestBranchCommand(Swerve swerve, SwerveRequest.FieldCentric fieldCentric, DoubleSupplier elevatorHeight) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        this.elevatorHeight = elevatorHeight;

        targetPose = new Pose2d();
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
        targetPose = FieldUtil.Reef.getNearestRobotPoseAtBranch(swerve.localizer.getStrategyPose());
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        double xError = currentPose.getX() - targetPose.getX();
        double yError = currentPose.getY() - targetPose.getY();
        double yawError = MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180);

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity) // TODO SHOP: TEST CLOSED LOOP
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(determineVelocity(xError, xPosDone))
                        .withVelocityY(determineVelocity(yError, yPosDone))
                        .withRotationalRate(yawController.calculate(
                                yawError,
                                0.0
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble()))
        );

        xPosDone = Math.abs(xError) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yPosDone = Math.abs(yError) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yawDone = Math.abs(yawError) < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;

        if (xPosDone && yPosDone && yawDone) {
            swerve.forceStop();
            swerve.consistentHeading = currentPose.getRotation().getDegrees();
            end = true;
        }
    }

    public double determineVelocity(double error, boolean done) {
        if (done) {
            return 0.0;
        }
        if (error < 0) {
            return ExpUtil.output(Math.abs(error), 0.02, 50);
        }
        return -ExpUtil.output(Math.abs(error), 0.02, 50);
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
