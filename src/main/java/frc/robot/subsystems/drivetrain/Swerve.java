package frc.robot.subsystems.drivetrain;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autos.Pathfinder;
import frc.robot.commands.DirectMoveToNearestBranchCommand;
import frc.robot.commands.auto.SearchForAlgaeCommand;
import frc.robot.constants.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DirectMoveToObjectCommand;
import frc.robot.subsystems.vision.Localizer;
import frc.robot.util.vision.PhotonUtil;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    public enum DriveMode {
        IDLE,
        ROTATING,
        FAST_ROTATING,
        TRANSLATING,
        BRANCH_HEADING,
        BRANCH_L1_HEADING,
        REEF_TAG_HEADING,
        REEF_TAG_OPPOSITE_HEADING,
        OBJECT_HEADING,
        CORAL_STATION_HEADING,
        PROCESSOR_HEADING,
        NET_HEADING
    }

    private DriveMode currentMode;

    /* An extension to the Swerve subsystem */
    public final Localizer localizer = new Localizer(this);
    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(this);

    public final Orchestra orchestra = new Orchestra();

    /* Swerve Command Requests */
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.SwerveDriveBrake xMode = new SwerveRequest.SwerveDriveBrake();

    private boolean hasAppliedDefaultRotation; // Keep track if we've ever applied the operator perspective before or not
    private boolean autoHeading;
    public double consistentHeading; // Heading to keep while translating without rotating

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

        currentMode = DriveMode.IDLE;

        Song.playRandom(this, Song.startupSongs);

        if (Utils.isSimulation()) {
            new SwerveSim(this).startSimThread();
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

        hasAppliedDefaultRotation = false;
        autoHeading = true;
        consistentHeading = 0.0;
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
            DoubleSupplier elevatorHeight,
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rotJoystick,
            DoubleSupplier rotLeft,
            DoubleSupplier rotRight,
            BooleanSupplier fastRotLeft,
            BooleanSupplier fastRotRight
    ) {
        return new DriveCommand(
                this,
                fieldCentric,
                elevatorHeight,
                straight,
                strafe,
                rotJoystick,
                rotLeft,
                rotRight,
                fastRotLeft,
                fastRotRight,
                () -> currentMode,
                () -> autoHeading
        );
    }

    public Command pathFindFindScoreAlgae(BooleanSupplier algaeObtained) {
        return new SearchForAlgaeCommand(this, fieldCentric)
                .andThen(directMoveToObject(algaeObtained, PhotonUtil.Color.TargetClass.ALGAE))
                .andThen(Commands.defer(
                        () -> Pathfinder.pathFindToNearestAlgaeScoringLocation(localizer.getStrategyPose()),
                        Set.of(this)
                ));
    }

    public Command pathFindToNearestBranch(DoubleSupplier elevatorHeight) {
        return Commands.defer(
                () -> Pathfinder.pathFindToNearestCoralScoringLocation(localizer.getStrategyPose()),
                Set.of(this)
        ).andThen(directMoveToNearestBranch(elevatorHeight));
    }

    public Command directMoveToObject(BooleanSupplier objectObtained, PhotonUtil.Color.TargetClass objectLabelClass) {
        return new DirectMoveToObjectCommand(this, robotCentric, objectObtained, objectLabelClass);
    }

    public Command directMoveToNearestBranch(DoubleSupplier elevatorHeight) {
        return new DirectMoveToNearestBranchCommand(this, fieldCentric, elevatorHeight);
    }

    public boolean isFullyTeleop() {
        return currentMode == DriveMode.IDLE
                || currentMode == DriveMode.ROTATING
                || currentMode == DriveMode.FAST_ROTATING
                || currentMode == DriveMode.TRANSLATING;
    }

    public boolean isAutoHeading() {
        return autoHeading;
    }

    public void forceStop() {
        setControl(fieldCentric
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
    }

    public void toggleAutoHeading() {
        autoHeading = !autoHeading;
    }

    public void setIdleMode() {
        currentMode = DriveMode.IDLE;
    }

    public void setRotatingMode() {
        currentMode = DriveMode.ROTATING;
    }

    public void setFastRotatingMode() {
        currentMode = DriveMode.FAST_ROTATING;
    }

    public void setTranslatingMode() {
        currentMode = DriveMode.TRANSLATING;
    }

    public void setBranchHeadingMode() {
        currentMode = DriveMode.BRANCH_HEADING;
    }

    public void setBranchHeadingL1Mode() {
        currentMode = DriveMode.BRANCH_L1_HEADING;
    }

    public void setReefTagHeadingMode() {
        currentMode = DriveMode.REEF_TAG_HEADING;
    }

    public void setReefTagOppositeHeadingMode() {
        currentMode = DriveMode.REEF_TAG_OPPOSITE_HEADING;
    }

    public void setObjectHeadingMode() {
        currentMode = DriveMode.OBJECT_HEADING;
    }

    public void setCoralStationHeadingMode() {
        currentMode = DriveMode.CORAL_STATION_HEADING;
    }

    public void setProcessorHeadingMode() {
        currentMode = DriveMode.PROCESSOR_HEADING;
    }

    public void setNetHeadingMode() {
        currentMode = DriveMode.NET_HEADING;
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
        if (!hasAppliedDefaultRotation || DriverStation.isDisabled()) {
            setOperatorPerspectiveForward(
                    Constants.ALLIANCE_SUPPLIER.get() == Alliance.Blue
                            ? Constants.BLUE_DEFAULT_ROTATION
                            : Constants.RED_DEFAULT_ROTATION
            );
            hasAppliedDefaultRotation = true;
        }

        if (DriverStation.isDisabled()) {
            localizer.syncRotations();
        }

        if (DriverStation.isDisabled() && !orchestra.isPlaying() && !Song.tenSeconds()) {
            Song.playRandom(this, Song.disableSongs);
        } if (!DriverStation.isDisabled() && orchestra.isPlaying() || Song.tenSeconds()) {
            orchestra.stop();
        }

        swerveTelemetry.publishValues();
        localizer.periodic();
    }
}
