package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.telemetry.SwerveTelemetry;
import frc.robot.telemetry.VisionTelemetry;
import frc.robot.util.VisionUtil;
import frc.robot.util.Simulator;
import frc.robot.util.TagLocation;
import org.photonvision.PhotonPoseEstimator;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final Simulator sim = new Simulator(this);

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Constants.MAX_VEL);
    private final VisionTelemetry visionTelemetry = new VisionTelemetry(this);

    // TODO TEST WITH REGULAR PID CONTROLLER
    private final PhoenixPIDController yawController;

    // TODO LOCALIZER "SUBSYSTEM" FOR SIMPLICITY, THEN A SUPPLIER FOR SWERVE MODULE STATES FOR AN UPDATE METHOD TO BE CALLED IN SWERVE PERIODIC
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            this.getKinematics(),
            this.getState().RawHeading,
            this.getState().ModulePositions,
            this.getState().Pose,
            Constants.VisionConstants.ODOM_STD_DEV,
            Constants.VisionConstants.VISION_STD_DEV
    );

    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            VisionUtil.Photon.BW.tagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionUtil.Photon.BW.cameraTransform
    );

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

        VisionUtil.Limelight.configureRobotToCameraTransform();

        configureSwerveUtils();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getTranslationToSpeaker() {
        Translation2d robotTranslation = getEstimatedPose().getTranslation();
        Translation2d tagTranslation = TagLocation.getSpeakerTagPose().getTranslation();
        if (tagTranslation.getNorm() == 0) { return new Translation2d(); }
        return tagTranslation.minus(robotTranslation);
    }

    public double getAngleToSpeaker() {
        return getTranslationToSpeaker().getAngle().getDegrees();
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
                                    getEstimatedPose().getRotation().getDegrees(),
                                    getAngleToSpeaker(),
                                    Timer.getFPGATimestamp() // TODO TEST this.getPigeon2().getYaw().getTimestamp().getTime() (TEST FOR CENTER ON NOTE TOO)
                            ) * Constants.MAX_ANGULAR_VEL
                        )
        );
    }

    public Command centerOnNote(DoubleSupplier straight, DoubleSupplier strafe) {
        double currentYaw = getEstimatedPose().getRotation().getDegrees();
        return applyRequest(() ->
                new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.MAX_VEL * 0.1)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                        .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                        .withRotationalRate(yawController.calculate(
                                        currentYaw,
                                        currentYaw - VisionUtil.Photon.Color.getBestObjectYaw(),
                                        Timer.getFPGATimestamp()
                                ) * Constants.MAX_ANGULAR_VEL
                        )
        );
    }

    public void moveToNote() { // TODO IMPLEMENT THIS AFTER CALIBRATING AUTO
    }

    public Command xMode() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    public void setPoses(Pose2d pose) {
        this.resetPose(pose);
        poseEstimator.resetPose(pose);
        VisionUtil.QuestNav.setPose(pose);
    }

    public void configureSwerveUtils() {
        VisionUtil.QuestNav.setOffset();
        registerTelemetry(swerveTelemetry::telemeterize);
        if (Utils.isSimulation()) {
            sim.startSimThread();
        }
    }

    public void updatePoses() {
        poseEstimator.update(this.getState().RawHeading, this.getState().ModulePositions);
        Pose2d limelightPose = VisionUtil.Limelight.getMegaTagOnePose();
        Pose2d photonPose = VisionUtil.Photon.BW.getPhotonPose();
        if (VisionUtil.Limelight.isTagClear()) {
            poseEstimator.addVisionMeasurement(
                    limelightPose,
                    Timer.getFPGATimestamp() - VisionUtil.Limelight.getLatency()
            );
        }
        if (VisionUtil.Photon.BW.isTagClear()) {
            poseEstimator.addVisionMeasurement(
                    photonPose,
                    VisionUtil.Photon.BW.getTimestamp()
            );
        }

        VisionUtil.QuestNav.updateOffset();
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
        updatePoses();
        visionTelemetry.publishValues();
    }
}
