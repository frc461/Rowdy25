package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.telemetry.SwerveTelemetry;
import frc.robot.util.VisionUtil;
import frc.robot.util.Simulator;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final Localizer localizer = new Localizer(this);
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(this);

    // TODO TEST WITH REGULAR PID CONTROLLER
    private final PhoenixPIDController yawController;
    private final PhoenixPIDController objectDetectionController;
    private final PIDController driveController;
    private final PIDController steerController;

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedDefaultRotation = false;

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

        yawController = new PhoenixPIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                Constants.SwerveConstants.ANGULAR_POSITION_I,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);


        objectDetectionController = new PhoenixPIDController(
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_P,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_I,
                Constants.SwerveConstants.ANGULAR_OBJECT_DETECTION_D
        );
        objectDetectionController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        driveController = new PIDController(
                Constants.SwerveConstants.DRIVE_GAINS.kP,
                Constants.SwerveConstants.DRIVE_GAINS.kI,
                Constants.SwerveConstants.DRIVE_GAINS.kD
        );

        steerController = new PIDController(
                Constants.SwerveConstants.STEER_GAINS.kP,
                Constants.SwerveConstants.STEER_GAINS.kI,
                Constants.SwerveConstants.STEER_GAINS.kD
        );
        steerController.enableContinuousInput(-Math.PI, Math.PI);

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
        return applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_VEL * 0.1) // Add a 10% deadband
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL) // Drive forward with negative Y (forward)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL) // Drive left with negative X (left)
                        .withRotationalRate(-rot.getAsDouble() * Constants.MAX_ANGULAR_VEL) // Drive counterclockwise with negative X (left)
        );
    }

    public Command driveTurret(DoubleSupplier straight, DoubleSupplier strafe) {
        return applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                        .withRotationalRate(
                            yawController.calculate(
                                    localizer.getStrategyPose().getRotation().getDegrees(),
                                    localizer.getAngleToSpeaker(),
                                    Timer.getFPGATimestamp()
                            ) * Constants.MAX_ANGULAR_VEL
                        )
        );
    }

    public Command centerOnNote(DoubleSupplier straight, DoubleSupplier strafe) {
        return applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                        .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                                ? objectDetectionController.calculate(
                                        0,
                                        -VisionUtil.Photon.Color.getBestObjectYaw(),
                                        Timer.getFPGATimestamp()
                                ) * Constants.MAX_ANGULAR_VEL
                                : 0.0
                        )
        );
    }

    public Command moveToNote() { // TODO CALIBRATE THIS AFTER CALIBRATING AUTO
        return applyRequest(() ->
                new SwerveRequest.RobotCentric()
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(VisionUtil.Photon.Color.hasTargets()
                            ? driveController.calculate(
                                0,
                                -VisionUtil.Photon.Color.getBestObjectPitch()
                            ) * Constants.MAX_VEL
                            : 0.0

                        )
                        .withVelocityY(0.0)
                        .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                                ? yawController.calculate(
                                        0,
                                        -VisionUtil.Photon.Color.getBestObjectYaw(),
                                        Timer.getFPGATimestamp()
                                ) * Constants.MAX_ANGULAR_VEL
                                : 0.0
                        )
        );

    }

    public Command xMode() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = localizer.getStrategyPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + driveController.calculate(pose.getX(), sample.x),
                sample.vy + driveController.calculate(pose.getY(), sample.y),
                sample.omega + steerController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        setControl(new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    public void toggleLocalizationStrategy() {
        localizer.toggleStrategy();
    }

    public void recalibrateMegaTag() {
        localizer.recalibrateMegaTag();
    }

    public Localizer getLocalizer() {
        return localizer;
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
