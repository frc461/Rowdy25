package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.telemetry.SwerveTelemetry;
import frc.robot.util.LimelightUtil;
import frc.robot.util.Simulator;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private final Simulator sim = new Simulator(this);

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
                Constants.VisionConstants.ODOM_UPDATE_FREQ,
                Constants.VisionConstants.ODOM_STD_DEV,
                Constants.VisionConstants.VISION_STD_DEV,
                Constants.SwerveConstants.FrontLeft.FRONT_LEFT,
                Constants.SwerveConstants.FrontRight.FRONT_RIGHT,
                Constants.SwerveConstants.BackLeft.BACK_LEFT,
                Constants.SwerveConstants.BackRight.BACK_RIGHT
        );
        configureSwerveUtils();
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

    public Command xMode() {
        return applyRequest(SwerveRequest.SwerveDriveBrake::new);
    }

    public void configureSwerveUtils() {
        registerTelemetry(new SwerveTelemetry(Constants.MAX_VEL)::telemeterize);
        if (Utils.isSimulation()) {
            sim.startSimThread();
        }
    }

    public void updateFusedPose() {
        Pose2d limelightPose = LimelightUtil.getMegaTagOnePose();
        if (LimelightUtil.tagExists() && LimelightUtil.getNearestTagDist() < 2.0) {
            this.addVisionMeasurement(
                    limelightPose,
                    Timer.getFPGATimestamp() - LimelightUtil.getLatency() / 1000.0
            );
        }
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
        updateFusedPose();
    }
}
