package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.VisionUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final PIDController objectDetectionController;
    private final PIDController headingController;
    private final DoubleConsumer setConsistentHeading;
    private final DoubleSupplier consistentHeading;
    private final DoubleSupplier straight;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;
    private final BooleanSupplier tagTurret;
    private final BooleanSupplier objectTurret;

    public DriveCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleConsumer setConsistentHeading,
            DoubleSupplier consistentHeading,
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rot,
            BooleanSupplier tagTurret,
            BooleanSupplier objectTurret
    ) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        objectDetectionController = new PIDController(
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P,
                0,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D
        );
        objectDetectionController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        headingController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                0
        );
        headingController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        this.setConsistentHeading = setConsistentHeading;
        this.consistentHeading = consistentHeading;
        this.straight = straight;
        this.strafe = strafe;
        this.rot = rot;
        this.tagTurret = tagTurret;
        this.objectTurret = objectTurret;
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        if (tagTurret.getAsBoolean()) {
            setConsistentHeading.accept(swerve.localizer.getStrategyPose().getRotation().getDegrees());
            swerve.setControl(
                    fieldCentric.withDeadband(Constants.MAX_VEL * 0.1)
                            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                            .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                            .withRotationalRate(
                                    yawController.calculate(
                                            swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                                            swerve.localizer.getAngleToSpeaker()
                                    ) * Constants.SwerveConstants.MAX_CONTROLLED_ANGULAR_VEL
                            )
            );
        } else if (objectTurret.getAsBoolean()) {
            setConsistentHeading.accept(swerve.localizer.getStrategyPose().getRotation().getDegrees());
            swerve.setControl(
                    fieldCentric
                            .withDeadband(Constants.MAX_VEL * 0.1)
                            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                            .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                            .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                                    ? objectDetectionController.calculate(
                                            VisionUtil.Photon.Color.getBestObjectYaw(),
                                            0
                                    ) * Constants.SwerveConstants.MAX_CONTROLLED_ANGULAR_VEL
                                            : 0.0
                            )
            );
        } else if (Math.abs(rot.getAsDouble()) >= 0.1 || (Math.abs(straight.getAsDouble()) < 0.1 && Math.abs(strafe.getAsDouble()) < 0.1)) {
            setConsistentHeading.accept(swerve.localizer.getStrategyPose().getRotation().getDegrees());
            swerve.setControl(
                    fieldCentric.withDeadband(Constants.MAX_VEL * 0.1)
                            .withRotationalDeadband(Constants.MAX_ANGULAR_VEL * 0.1) // Add a 10% deadband
                            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
                            .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL) // Drive forward with negative Y (forward)
                            .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL) // Drive left with negative X (left)
                            .withRotationalRate(-rot.getAsDouble() * Constants.MAX_ANGULAR_VEL) // Drive counterclockwise with negative X (left)
            );
        } else {
            swerve.setControl(
                    fieldCentric.withDeadband(Constants.MAX_VEL * 0.1)
                            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                            .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                            .withRotationalRate(headingController.calculate(
                                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                                    consistentHeading.getAsDouble()
                            ) * Constants.SwerveConstants.MAX_CONTROLLED_ANGULAR_VEL)
            );
        }
    }
}
