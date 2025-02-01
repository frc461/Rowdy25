package frc.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autos.PathManager;
import frc.robot.commands.DirectAlignToNearestBranchCommand;
import frc.robot.commands.auto.SearchForAlgaeCommand;
import frc.robot.constants.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveToObjectCommand;
import frc.robot.constants.variants.DefaultConstants;
import frc.robot.subsystems.vision.Localizer;
import frc.robot.util.Elastic;
import frc.robot.util.FieldUtil;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    /* An extension to the Swerve subsystem */
    public final Localizer localizer = new Localizer(this);

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(this);

    public final Orchestra orchestra = new Orchestra();

    /* Swerve Command Requests */
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake xMode = new SwerveRequest.SwerveDriveBrake();

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedDefaultRotation = false;

    public double consistentHeading = 0.0;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     */
    public Swerve() {
        /* ah, */ super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                Constants.SwerveConstants.SWERVE_DRIVETRAIN_CONSTANTS,
                Constants.SwerveConstants.FRONT_LEFT,
                Constants.SwerveConstants.FRONT_RIGHT,
                Constants.SwerveConstants.BACK_LEFT,
                Constants.SwerveConstants.BACK_RIGHT
        );

        if (Utils.isSimulation()) {
            new SwerveSim(this).startSimThread();
        }

        Arrays.stream(this.getModules()).forEach(
                module -> {
                    module.getDriveMotor().getConfigurator().apply(Constants.SwerveConstants.AUDIO_CONFIGS, 0.05);
                    module.getSteerMotor().getConfigurator().apply(Constants.SwerveConstants.AUDIO_CONFIGS, 0.05);

                    orchestra.addInstrument(module.getDriveMotor());
                    orchestra.addInstrument(module.getSteerMotor());
                }
        );

        StatusCode status = orchestra.loadMusic("sound/mario.chrp");

        Elastic.Notification.NotificationLevel notificationLevel;
        if (status.isWarning()) {
            notificationLevel = Elastic.Notification.NotificationLevel.WARNING;
        } else if (status.isError()) {
            notificationLevel = Elastic.Notification.NotificationLevel.ERROR;
        } else {
            notificationLevel = Elastic.Notification.NotificationLevel.INFO;
        }

        Elastic.sendNotification(
                new Elastic.Notification(
                        notificationLevel,
                        "Orchestra status",
                        status.getName() + ": " + status.getDescription()
                )
        );

        orchestra.play();

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
            BooleanSupplier objectTurret,
            BooleanSupplier coralStationTurret,
            BooleanSupplier algaeScoringTurret
    ) {
        return new DriveCommand(
                this,
                fieldCentric,
                straight,
                strafe,
                rotLeft,
                rotRight,
                tagTurret,
                objectTurret,
                coralStationTurret,
                algaeScoringTurret
        );
    }

    public Command pathFindFindScoreAlgae() {
        return new SearchForAlgaeCommand(this, fieldCentric)
                .andThen(new DriveToObjectCommand(this, robotCentric))
                .andThen(Commands.defer(
                        () -> PathManager.pathFindToNearestScoringLocation(localizer.getStrategyPose()),
                        Set.of(this)
                ));
    }

    public Command moveToObject() {
        return new DriveToObjectCommand(this, robotCentric);
    }

    public Command moveToNearestBranch() {
        return Commands.defer(() -> {
            Pose2d nearestBranchPose = FieldUtil.Coral.getNearestBranchPose(localizer.getStrategyPose());
            return PathManager.pathFindToClosePose(
                    localizer.getStrategyPose(),
                    new Pose2d(
                            nearestBranchPose.getTranslation(),
                            nearestBranchPose.getRotation().rotateBy(Rotation2d.kPi)
                    ),
                    nearestBranchPose.getRotation().rotateBy(Rotation2d.fromDegrees(-80)),
                    nearestBranchPose.getRotation().rotateBy(Rotation2d.fromDegrees(80)),
                    Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO,
                    2.0
            );
        }, Set.of(this)).andThen(directAlignToNearestBranch());
    }

    public Command directAlignToNearestBranch() {
        return new DirectAlignToNearestBranchCommand(this, fieldCentric);
    }

    public Command xMode() {
        return applyRequest(() -> xMode);
    }

    public Command resetGyro() {
        return runOnce(() -> resetRotation(localizer.getStrategyPose().getRotation()));
    }

    public void forceStop() {
        setControl(fieldCentric
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
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
        // TODO THIS IS A PROBLEM (CAUSES APPLYING STATES ON RED TEAM TO BE BACKWARD)
        if ((!hasAppliedDefaultRotation || DriverStation.isDisabled()) && Constants.ALLIANCE_SUPPLIER.get() != null) {
            setOperatorPerspectiveForward(
                    Constants.ALLIANCE_SUPPLIER.get() == Alliance.Blue
                            ? Constants.BLUE_DEFAULT_ROTATION
                            : Constants.RED_DEFAULT_ROTATION
            );
            hasAppliedDefaultRotation = true;
        }

        if (!DriverStation.isDisabled()) {
            orchestra.stop();
        }

        swerveTelemetry.publishValues();
        localizer.periodic();
    }
}
