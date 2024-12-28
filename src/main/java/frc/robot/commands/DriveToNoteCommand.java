package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.VisionUtil;

public class DriveToNoteCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.RobotCentric robotCentric;
    private final PIDController objectDetectionController;
    private boolean rotationComplete;
    private boolean translationComplete;

    public DriveToNoteCommand(Swerve swerve, SwerveRequest.RobotCentric robotCentric) {
        this.swerve = swerve;
        this.robotCentric = robotCentric;

        objectDetectionController = new PIDController(
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P,
                0,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D
        );
        objectDetectionController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        rotationComplete = false;
        translationComplete = false;

        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        boolean targetValid = VisionUtil.Photon.Color.hasTargets();
        double currentYaw = VisionUtil.Photon.Color.getBestObjectYaw();
        double currentPitch = VisionUtil.Photon.Color.getBestObjectPitch();
        if (targetValid && !rotationComplete) {
            double degreeError = Math.abs(currentYaw);

            swerve.setControl(
                    robotCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withRotationalRate(objectDetectionController.calculate(
                                    currentYaw,
                                    0
                            ) * Constants.SwerveConstants.MAX_CONTROLLED_ANGULAR_VEL)
            );
            if (degreeError < Constants.VisionConstants.PhotonConstants.OBJECT_DEGREE_TOLERANCE_TO_ACCEPT) {
                rotationComplete = true;
            }
        } else if (targetValid && !translationComplete) {
            double degreeError = Math.abs(currentPitch - Constants.VisionConstants.PhotonConstants.OBJECT_GOAL_PITCH);

            swerve.setControl(
                    robotCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(objectDetectionController.calculate(
                                    currentPitch,
                                    Constants.VisionConstants.PhotonConstants.OBJECT_GOAL_PITCH
                            ) * Constants.MAX_VEL)
                            .withVelocityY(objectDetectionController.calculate(
                                    -currentYaw,
                                    0
                            ))
            );
            if (degreeError < Constants.VisionConstants.PhotonConstants.OBJECT_DEGREE_TOLERANCE_TO_ACCEPT) {
                translationComplete = true;
            }
        } else {
            swerve.setControl(
                    robotCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(0.0)
                            .withVelocityY(0.0)
                            .withRotationalRate(0.0)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        rotationComplete = false;
        translationComplete = false;
    }
}