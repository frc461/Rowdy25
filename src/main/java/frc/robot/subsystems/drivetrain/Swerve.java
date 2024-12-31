package frc.robot.subsystems.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToNoteCommand;
import frc.robot.subsystems.vision.Localizer;
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
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake xMode = new SwerveRequest.SwerveDriveBrake();

    /* PID Controllers */
    private final PIDController pathTranslationController;
    private final PIDController pathSteeringController;

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

        AutoBuilder.configure(
                localizer::getStrategyPose,
                localizer::setPoses,
                () -> getKinematics().toChassisSpeeds(getState().ModuleStates),
                (speeds, feedforwards) -> setControl(new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
                ),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P,
                                0,
                                0
                        ),
                        new PIDConstants(
                                Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P,
                                0,
                                0
                        )
                ),
                Constants.AutoConstants.ROBOT_CONFIG,
                () -> Constants.ALLIANCE_SUPPLIER.get() == Alliance.Red,
                this
        );
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

    public Command driveFieldCentric(
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rotLeft,
            DoubleSupplier rotRight,
            BooleanSupplier tagTurret,
            BooleanSupplier objectTurret
    ) {
        return new DriveCommand(
                this,
                fieldCentric,
                heading -> consistentHeading = heading,
                () -> consistentHeading,
                straight,
                strafe,
                rotLeft,
                rotRight,
                tagTurret,
                objectTurret
        );
    }

    public Command moveToNote() { // TODO IMPLEMENT THIS AFTER CALIBRATING AUTO
        return new DriveToNoteCommand(this, robotCentric);
    }

    public Command xMode() {
        return applyRequest(() -> xMode);
    }

    public Command resetGyro() {
        return runOnce(() -> {
                seedFieldCentric();
                localizer.setPoses(new Pose2d(
                        localizer.getStrategyPose().getTranslation(),
                        new Rotation2d()
                ));
        });
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
