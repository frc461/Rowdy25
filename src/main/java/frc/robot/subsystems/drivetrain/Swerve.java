package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.DriveConsistentHeadingCommand;
import frc.robot.telemetry.SwerveTelemetry;
import frc.robot.util.VisionUtil;
import frc.robot.util.Simulator;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    /* An extension to the Swerve subsystem */
    public final Localizer localizer = new Localizer(this);

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(this);

    /* Swerve Command Requests */
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.SwerveDriveBrake xMode = new SwerveRequest.SwerveDriveBrake();

    /* PID Controllers */
    private final PIDController pathTranslationController;
    private final PIDController pathSteeringController;
    private final PIDController yawController;
    private final PIDController objectDetectionController;

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedDefaultRotation = false;

    private double consistentHeading = 0.0;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     */
    public Swerve() {
        super(
                Constants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS,
                Constants.SwerveConstants.FrontLeft.FRONT_LEFT,
                Constants.SwerveConstants.FrontRight.FRONT_RIGHT,
                Constants.SwerveConstants.BackLeft.BACK_LEFT,
                Constants.SwerveConstants.BackRight.BACK_RIGHT
        );

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                Constants.SwerveConstants.ANGULAR_POSITION_I,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);


        objectDetectionController = new PIDController(
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_I,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D
        );
        objectDetectionController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        pathTranslationController = new PIDController(
                Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P,
                0,
                0
        );

        pathSteeringController = new PIDController(
                Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P,
                0,
                0
        );
        pathSteeringController.enableContinuousInput(-Math.PI, Math.PI);

        if (Utils.isSimulation()) {
            new Simulator(this).startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command driveFieldCentric(DoubleSupplier straight, DoubleSupplier strafe, DoubleSupplier rot) {
        return new DriveConsistentHeadingCommand( // TODO TEST THIS COMMAND
                this,
                fieldCentric,
                yawController,
                heading -> consistentHeading = heading,
                () -> consistentHeading,
                straight,
                strafe,
                rot
        );
    }

    public Command driveTurret(DoubleSupplier straight, DoubleSupplier strafe) {
        return applyRequest(() -> fieldCentric
                .withDeadband(Constants.MAX_VEL * 0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                .withRotationalRate(
                    yawController.calculate(
                            localizer.getStrategyPose().getRotation().getDegrees(),
                            localizer.getAngleToSpeaker()
                    ) * Constants.MAX_ANGULAR_VEL
                )
        );
    }

    public Command centerOnNote(DoubleSupplier straight, DoubleSupplier strafe) {
        return applyRequest(() -> fieldCentric
                .withDeadband(Constants.MAX_VEL * 0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                        ? objectDetectionController.calculate(
                                0,
                                -VisionUtil.Photon.Color.getBestObjectYaw()
                        ) * Constants.MAX_ANGULAR_VEL
                        : 0.0
                )
        );
    }

    public Command moveToNote() { // TODO IMPLEMENT THIS AFTER CALIBRATING AUTO
        return applyRequest(() -> fieldCentric
                .withDeadband(Constants.MAX_VEL * 0.1)
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withVelocityX(VisionUtil.Photon.Color.hasTargets()
                    ? pathTranslationController.calculate(
                        0,
                        -VisionUtil.Photon.Color.getBestObjectPitch()
                    ) * Constants.MAX_VEL
                    : 0.0

                )
                .withVelocityY(0.0)
                .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                        ? yawController.calculate(
                                0,
                                -VisionUtil.Photon.Color.getBestObjectYaw()
                        ) * Constants.MAX_ANGULAR_VEL
                        : 0.0
                )
        );

    }

    public Command xMode() {
        return applyRequest(() -> xMode);
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = localizer.getStrategyPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + pathTranslationController.calculate(pose.getX(), sample.x),
                sample.vy + pathTranslationController.calculate(pose.getY(), sample.y),
                sample.omega + pathSteeringController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if ((!hasAppliedDefaultRotation || DriverStation.isDisabled()) && Constants.ALLIANCE_SUPPLIER.get() != null) {
            setOperatorPerspectiveForward(
                    Constants.ALLIANCE_SUPPLIER.get() == Alliance.Blue
                            ? Constants.BLUE_DEFAULT_ROTATION
                            : Constants.RED_DEFAULT_ROTATION
            );
            hasAppliedDefaultRotation = true;
        }
        swerveTelemetry.publishValues();
        localizer.periodic();
    }
}
