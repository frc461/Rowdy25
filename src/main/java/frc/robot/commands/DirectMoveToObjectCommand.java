package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.vision.PhotonUtil;

import java.util.function.BooleanSupplier;

public class DirectMoveToObjectCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.RobotCentric robotCentric;
    private final PIDController objectDetectionController;
    private final BooleanSupplier objectObtained;
    private final PhotonUtil.Color.TargetClass objectClass;
    private boolean rotationComplete;
    private boolean translationComplete;
    private boolean end;

    public DirectMoveToObjectCommand(Swerve swerve, SwerveRequest.RobotCentric robotCentric, BooleanSupplier objectObtained, PhotonUtil.Color.TargetClass objectClass) {
        this.swerve = swerve;
        this.robotCentric = robotCentric;
        this.objectObtained = objectObtained;
        this.objectClass = objectClass;

        objectDetectionController = new PIDController(
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P,
                0,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D
        );
        objectDetectionController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        rotationComplete = false;
        translationComplete = false;
        end = false;
    }

    @Override
    public void execute() {
        boolean targetValid = PhotonUtil.Color.hasTargets(objectClass);
        double currentYaw = PhotonUtil.Color.getBestObjectYaw(objectClass);
        double currentPitch = PhotonUtil.Color.getBestObjectPitch(objectClass);
        if (targetValid && !rotationComplete) {
            double yawError = Math.abs(currentYaw);

            swerve.setControl(
                    robotCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity) // TODO SHOP: TEST CLOSED LOOP
                            .withVelocityX(0.0)
                            .withVelocityY(0.0)
                            .withRotationalRate(objectDetectionController.calculate(
                                    currentYaw,
                                    0.0
                            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(0.0))
            );
            if (yawError < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT) {
                rotationComplete = true;
            }
        } else if (targetValid && !translationComplete) {
            double yawError = Math.abs(currentYaw);
            double pitchError = Math.abs(currentPitch - Constants.VisionConstants.PhotonConstants.OBJECT_TARGET_PITCH);

            swerve.setControl(
                    robotCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(objectDetectionController.calculate(
                                    currentPitch,
                                    Constants.VisionConstants.PhotonConstants.OBJECT_TARGET_PITCH
                            ) * Constants.MAX_VEL)
                            .withVelocityY(objectDetectionController.calculate(
                                    -currentYaw,
                                    0.0
                            ))
                            .withRotationalRate(0.0)
            );
            if (yawError > Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT) {
                rotationComplete = false;
            } else if (pitchError < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT) {
                translationComplete = true;
                end = true;
            }
        } else {
            // TODO: MORE ROBUST CHECKING I.E., IF OBJECT INTAKE
            end = true;
        }

        if (objectObtained.getAsBoolean()) {
            end = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.forceStop();
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
