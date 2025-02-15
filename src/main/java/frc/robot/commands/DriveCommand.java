package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.VisionUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final PIDController objectDetectionController;
    private final PIDController headingController;
    private final DoubleSupplier straight;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;
    private final BooleanSupplier fastRotationLeft;
    private final BooleanSupplier fastRotationRight;
    private final DoubleSupplier elevatorHeight;
    private final Supplier<Swerve.DriveMode> driveMode;
    private final BooleanSupplier autoHeading;

    public DriveCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rotLeft,
            DoubleSupplier rotRight,
            BooleanSupplier fastRotationLeft,
            BooleanSupplier fastRotationRight,
            Supplier<Swerve.DriveMode> driveMode,
            BooleanSupplier autoHeading
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
        this.straight = straight;
        this.strafe = strafe;
        this.rot = () -> rotRight.getAsDouble() - rotLeft.getAsDouble();
        this.fastRotationLeft = fastRotationLeft;
        this.fastRotationRight = fastRotationRight;
        this.elevatorHeight = elevatorHeight;
        this.driveMode = driveMode;
        this.autoHeading = autoHeading;
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        updateMode();

        swerve.consistentHeading = driveMode.get() == Swerve.DriveMode.TRANSLATING
                ? swerve.consistentHeading
                : swerve.localizer.getStrategyPose().getRotation().getDegrees();

        swerve.setControl(
                fieldCentric.withDeadband(Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()) * Constants.DEADBAND)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(determineTranslationalRate(-straight.getAsDouble()))
                        .withVelocityY(determineTranslationalRate(-strafe.getAsDouble()))
                        .withRotationalRate(determineRotationalRate())
        );
    }

    private void updateMode() {
        if (swerve.isFullyTeleop() || !autoHeading.getAsBoolean()) {
            if (fastRotationLeft.getAsBoolean() || fastRotationRight.getAsBoolean()) {
                swerve.setFastRotatingMode();
            } else if (Math.abs(rot.getAsDouble()) >= Constants.DEADBAND) {
                swerve.setRotatingMode();
            } else if (Math.abs(straight.getAsDouble()) < Constants.DEADBAND && Math.abs(strafe.getAsDouble()) < Constants.DEADBAND) {
                swerve.setIdleMode();
            } else {
                swerve.setTranslatingMode();
            }
        }
    }

    private double determineTranslationalRate(double axis) {
        return switch (driveMode.get()) {
            case BRANCH_HEADING, REEF_TAG_HEADING, CORAL_STATION_HEADING, PROCESSOR_HEADING, NET_HEADING ->
                MathUtil.clamp(axis * Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()), -1.0, 1.0);
            case OBJECT_HEADING ->
                VisionUtil.Photon.Color.hasTargets()
                        ? MathUtil.clamp(axis * Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()), -1.0, 1.0)
                        : axis * Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble());
            default -> axis * Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble());
        };
    }

    private double determineRotationalRate() {
        return switch (driveMode.get()) {
            case IDLE, ROTATING -> -rot.getAsDouble() * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case FAST_ROTATING -> -rot.getAsDouble() * Constants.MAX_ANGULAR_VEL;
            case TRANSLATING -> headingController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.consistentHeading
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case BRANCH_HEADING -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getNearestReefSideHeading()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case REEF_TAG_HEADING -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getAngleToNearestReefSide()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case OBJECT_HEADING -> VisionUtil.Photon.Color.hasTargets()
                    ? objectDetectionController.calculate(
                            VisionUtil.Photon.Color.getBestObjectYaw(),
                            0
                    ) * Constants.MAX_CONTROLLED_ANGULAR_VEL
                    : -rot.getAsDouble() * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case CORAL_STATION_HEADING -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getNearestCoralStationHeading()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case PROCESSOR_HEADING -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getProcessorScoringHeading()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case NET_HEADING -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getNetScoringHeading()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
        };
    }
}
