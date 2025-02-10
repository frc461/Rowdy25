package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.VisionUtil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final PIDController objectDetectionController;
    private final PIDController headingController;
    private final DoubleSupplier straight;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;
    private final BooleanSupplier branchAlignment;
    private final BooleanSupplier objectAlignment;
    private final BooleanSupplier coralStationAlignment;
    private final BooleanSupplier algaeScoringAlignment;
    private DriveMode driveMode;

    public enum DriveMode {
        IDLE_OR_ROTATION,
        FAST_ROTATION,
        TRANSLATING,
        BRANCH_ALIGNMENT,
        OBJECT_ALIGNMENT,
        CORAL_STATION_ALIGNMENT,
        ALGAE_SCORING_ALIGNMENT
    }

    public DriveCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rotLeft,
            DoubleSupplier rotRight,
            BooleanSupplier branchAlignment,
            BooleanSupplier objectAlignment,
            BooleanSupplier coralStationAlignment,
            BooleanSupplier algaeScoringAlignment
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
        this.branchAlignment = branchAlignment;
        this.objectAlignment = objectAlignment;
        this.coralStationAlignment = coralStationAlignment;
        this.algaeScoringAlignment = algaeScoringAlignment;
        this.driveMode = DriveMode.IDLE_OR_ROTATION;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        driveMode = DriveMode.IDLE_OR_ROTATION;
    }

    @Override
    public void execute() {
        // TODO SHOP: FIGURE OUT AUTOMATION WITH SUPERSTRUCTURE
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        driveMode = branchAlignment.getAsBoolean() ?
                        rot.getAsDouble() < -0.5 ? DriveMode.FAST_ROTATION : DriveMode.BRANCH_ALIGNMENT
                : objectAlignment.getAsBoolean() ?
                        rot.getAsDouble() > 0.5 ? DriveMode.FAST_ROTATION : DriveMode.OBJECT_ALIGNMENT
                : coralStationAlignment.getAsBoolean() ? DriveMode.CORAL_STATION_ALIGNMENT
                : algaeScoringAlignment.getAsBoolean() ? DriveMode.ALGAE_SCORING_ALIGNMENT
                : Math.abs(rot.getAsDouble()) >= Constants.DEADBAND
                        || (Math.abs(straight.getAsDouble()) < Constants.DEADBAND && Math.abs(strafe.getAsDouble()) < Constants.DEADBAND) ? DriveMode.IDLE_OR_ROTATION
                : DriveMode.TRANSLATING;

        swerve.consistentHeading = driveMode == DriveMode.TRANSLATING ? swerve.consistentHeading : currentPose.getRotation().getDegrees();

        swerve.setControl(
                fieldCentric.withDeadband(Constants.MAX_VEL * Constants.DEADBAND)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                        .withRotationalRate(determineRotationalRate())
        );
    }

    private double determineRotationalRate() {
        return switch (driveMode) {
            case IDLE_OR_ROTATION -> -rot.getAsDouble() * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case FAST_ROTATION -> -rot.getAsDouble() * Constants.MAX_ANGULAR_VEL;
            case TRANSLATING -> headingController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.consistentHeading
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case BRANCH_ALIGNMENT -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getAngleToNearestBranch()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case OBJECT_ALIGNMENT -> VisionUtil.Photon.Color.hasTargets()
                    ? objectDetectionController.calculate(
                            VisionUtil.Photon.Color.getBestObjectYaw(),
                            0
                    ) * Constants.MAX_CONTROLLED_ANGULAR_VEL
                    : 0.0;
            case CORAL_STATION_ALIGNMENT -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getNearestCoralStationHeading()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
            case ALGAE_SCORING_ALIGNMENT -> yawController.calculate(
                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                    swerve.localizer.getNearestAlgaeScoringHeading()
            ) * Constants.MAX_CONTROLLED_ANGULAR_VEL;
        };
    }
}
