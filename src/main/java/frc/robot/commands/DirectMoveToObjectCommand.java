package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.FieldUtil;
import frc.robot.util.PhoenixProfiledPIDController;
import frc.robot.util.vision.PhotonUtil;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Meters;

public class DirectMoveToObjectCommand extends Command {
    public enum CommandStage {
        TO_OBJECT,
        SEARCH,
        WAIT
    }

    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final PhoenixProfiledPIDController velocityController;
    private final BooleanSupplier objectObtained;
    private final PhotonUtil.Color.TargetClass objectClass;
    private final double maxVelocity;
    private Pose2d targetPose;
    private boolean xPosDone, yPosDone, yawDone, end;
    private CommandStage currentStage;

    public DirectMoveToObjectCommand( // TODO SHOP: TEST AND TUNE THIS CLASS, THEN IF IT WORKS THEN INTEGRATE INTO AUTOMATION
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            BooleanSupplier objectObtained,
            PhotonUtil.Color.TargetClass objectClass,
            double maxVelocity
    ) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        velocityController = new PhoenixProfiledPIDController(
                Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P,
                0,
                Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_D,
                new TrapezoidProfile.Constraints(
                        Constants.MAX_VEL,
                        Constants.MAX_ACCEL
                )
        );

        this.objectObtained = objectObtained;
        this.objectClass = objectClass;
        this.maxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_VEL);
        targetPose = new Pose2d();
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        currentStage = CommandStage.TO_OBJECT;

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();

        if (PhotonUtil.Color.getRobotToBestObject(objectClass).isPresent()) {
            Transform2d robotToObject = PhotonUtil.Color.getRobotToBestObject(objectClass).get();
            targetPose = new Pose2d(
                    currentPose.plus(robotToObject).plus(new Transform2d(
                            Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2 + Units.inchesToMeters(12.0),
                            0,
                            Rotation2d.kZero
                    ).inverse()).getTranslation(),
                    robotToObject.getTranslation().getAngle()
            );

            velocityController.setGoal(new TrapezoidProfile.State(0.0, 0.0));
            velocityController.reset(
                    currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                    Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond),
                    swerve.getState().Timestamp
            );
            velocityController.setTolerance(Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT);

            xPosDone = false;
            yPosDone = false;
            yawDone = false;
            end = false;
            currentStage = CommandStage.TO_OBJECT;
        } else {
            end = true;
        }
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();

        switch (currentStage) {
            case TO_OBJECT:
                if (PhotonUtil.Color.getRobotToBestObject(objectClass).isPresent()) {
                    Transform2d robotToObject = PhotonUtil.Color.getRobotToBestObject(objectClass).get();
                    targetPose = new Pose2d(
                            currentPose.plus(robotToObject).plus(new Transform2d(
                                    Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2 + Units.inchesToMeters(12.0),
                                    0,
                                    Rotation2d.kZero
                            ).inverse()).getTranslation(),
                            robotToObject.getTranslation().getAngle()
                    );
                }
                break;
            case WAIT:
                if (PhotonUtil.Color.getRobotToBestObject(objectClass).isPresent()) {
                    currentStage = CommandStage.TO_OBJECT;
                    Transform2d robotToObject = PhotonUtil.Color.getRobotToBestObject(objectClass).get();
                    targetPose = new Pose2d(
                            currentPose.plus(robotToObject).plus(new Transform2d(
                                    Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2 + Units.inchesToMeters(12.0),
                                    0,
                                    Rotation2d.kZero
                            ).inverse()).getTranslation(),
                            robotToObject.getTranslation().getAngle()
                    );
                }

                velocityController.reset(
                        currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                        Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond),
                        swerve.getState().Timestamp
                );
                break;
        }

        swerve.localizer.setCurrentTemporaryTargetPose(targetPose);

        double velocity = Math.abs(velocityController.calculate(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                new TrapezoidProfile.Constraints(
                        maxVelocity,
                        Constants.MAX_ACCEL
                ),
                swerve.getState().Timestamp
        ));

        double velocityHeadingRadians = targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().getRadians();

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withDeadband(0.0)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(Math.cos(velocityHeadingRadians) * velocity)
                        .withVelocityY(Math.sin(velocityHeadingRadians) * velocity)
                        .withRotationalRate(yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                targetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(0.0))
        );

        xPosDone = Math.abs(currentPose.getX() - targetPose.getX())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yPosDone = Math.abs(currentPose.getY() - targetPose.getY())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yawDone = Math.abs(MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180))
                < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;

        if (xPosDone && yPosDone && yawDone) {
            switch (currentStage) {
                case TO_OBJECT:
                    currentStage = CommandStage.SEARCH;
                    Rotation2d reefToLastTargetLocation = FieldUtil.Reef.getReefCenter().minus(targetPose.getTranslation()).getAngle();
                    targetPose = new Pose2d(
                            currentPose.getTranslation().interpolate(FieldUtil.Reef.getReefCenter(), 0.2),
                            currentPose.getRotation().interpolate(reefToLastTargetLocation, 0.2)
                    );

                    velocityController.reset(
                            currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                            Math.hypot(swerve.getState().Speeds.vxMetersPerSecond, swerve.getState().Speeds.vyMetersPerSecond),
                            swerve.getState().Timestamp
                    );
                    break;
                case SEARCH:
                    currentStage = CommandStage.WAIT;
                    swerve.forceStop();
                    break;
            }
        }

        if (objectObtained.getAsBoolean()) {
            end = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.forceStop();
        swerve.consistentHeading = swerve.localizer.getStrategyPose().getRotation().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
