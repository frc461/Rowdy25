package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
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
    private final PIDController yawController;
    private final PIDController objectDetectionController;
    private final PIDController driveController;

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

        driveController = new PIDController(
            Constants.SwerveConstants.DRIVE_GAINS.kP,
            Constants.SwerveConstants.DRIVE_GAINS.kI,
            Constants.SwerveConstants.DRIVE_GAINS.kD
        );

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
        return ((applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_VEL * 0.1) // Add a 10% deadband
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL) // Drive forward with negative Y (forward)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL) // Drive left with negative X (left)
                        .withRotationalRate(-rot.getAsDouble() * Constants.MAX_ANGULAR_VEL)) // Drive counterclockwise with negative X (left)
                .alongWith(runOnce(() -> consistentHeading = this.localizer.getStrategyPose().getRotation().getDegrees()))).onlyIf(
                        () -> (Math.abs(straight.getAsDouble()) < 0.1 && Math.abs(strafe.getAsDouble()) < 0.1) || Math.abs(rot.getAsDouble()) >= 0.1
                )).alongWith(applyRequest(() ->
                        new SwerveRequest.FieldCentric()
                                .withDeadband(Constants.MAX_VEL * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_VEL * 0.1)
                                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                                .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                                .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                                .withRotationalRate(yawController.calculate(
                                        localizer.getStrategyPose().getRotation().getDegrees(),
                                        consistentHeading
                                ) * Constants.MAX_ANGULAR_VEL))
                        .onlyIf(() -> Math.abs(rot.getAsDouble()) < 0.1 && Math.abs(straight.getAsDouble()) >= 0.1 && Math.abs(strafe.getAsDouble()) >= 0.1));
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
                                    localizer.getAngleToSpeaker()
                            ) * Constants.MAX_ANGULAR_VEL
                        )
        );
    }

    public Command centerOnNote(DoubleSupplier straight, DoubleSupplier strafe) {
        double currentYaw = localizer.getStrategyPose().getRotation().getDegrees();
        return applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                        .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                                ? objectDetectionController.calculate(
                                        currentYaw,
                                        currentYaw - VisionUtil.Photon.Color.getBestObjectYaw()
                                ) * Constants.MAX_ANGULAR_VEL
                                : 0.0
                        )
        );
    }

    public Command moveToNote() { // TODO IMPLEMENT THIS AFTER CALIBRATING AUTO
        double currentYaw = localizer.getStrategyPose().getRotation().getDegrees();
        double currentPitch = 25; // TODO: MATCH REAL PITCH
        return applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(VisionUtil.Photon.Color.hasTargets()
                            ? driveController.calculate(
                                currentPitch,
                                currentPitch - VisionUtil.Photon.Color.getBestObjectPitch()
                            ) * Constants.MAX_VEL
                            : 0.0

                        )
                        .withVelocityY(0.0)
                        .withRotationalRate(VisionUtil.Photon.Color.hasTargets()
                                ? yawController.calculate(
                                        currentYaw,
                                        currentYaw - VisionUtil.Photon.Color.getBestObjectYaw()
                                ) * Constants.MAX_ANGULAR_VEL
                                : 0.0
                        )
        );

    }

    public Command xMode() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    public void toggleLocalizationStrategy() {
        localizer.toggleStrategy();
    }

    public void recalibrate() {
        localizer.recalibrateMegaTag();
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
