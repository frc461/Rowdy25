package io.github.frc461.rowdy25.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.subsystems.drivetrain.Swerve;
import io.github.frc461.rowdy25.util.EquationUtil;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;

import java.util.function.BooleanSupplier;

public class SearchForObjectCommand extends Command {
    public enum CommandStage {
        TO_OBJECT,
        SEARCH,
        WAIT
    }

    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final BooleanSupplier objectObtained;
    private final PhotonUtil.Color.TargetClass objectClass;
    private final double maxVelocity;
    private Pose2d targetPose;
    private boolean xPosDone, yPosDone, yawDone, end;
    private CommandStage currentStage;

    public SearchForObjectCommand( // TODO SHOP: TEST THIS
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
        PhotonUtil.Color.getRobotToBestObject(objectClass).ifPresentOrElse(
                robotToObject -> {
                    targetPose = swerve.localizer.bestCoralPose;
                    xPosDone = false;
                    yPosDone = false;
                    yawDone = false;
                    end = false;
                    currentStage = CommandStage.TO_OBJECT;
                },
                () -> end = true
        );
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();

        switch (currentStage) {
            case TO_OBJECT:
                PhotonUtil.Color.getRobotToBestObject(objectClass).ifPresent(robotToObject ->
                        targetPose = swerve.localizer.bestCoralPose
                );
                break;
            case WAIT:
                PhotonUtil.Color.getRobotToBestObject(objectClass).ifPresentOrElse(
                        robotToObject -> currentStage = CommandStage.TO_OBJECT,
                        () -> targetPose = currentPose
                );
                break;
        }

        if (currentStage == CommandStage.TO_OBJECT || currentStage == CommandStage.SEARCH) {
            swerve.localizer.setCurrentTemporaryTargetPose(targetPose);

            double velocity = Math.max(
                    EquationUtil.expOutput(
                            targetPose.getTranslation().getDistance(currentPose.getTranslation()),
                            2,
                            2 / 7.0,
                            15 / 2.0
                    ),
                    Math.min(EquationUtil.linearOutput(targetPose.getTranslation().getDistance(currentPose.getTranslation()), 10, -8), maxVelocity)
            );

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
        }

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
                    Translation2d nearestReefCenter = FieldUtil.Reef.getNearestReefCenter(currentPose.getTranslation());
                    Rotation2d reefToLastTargetLocation = nearestReefCenter.minus(targetPose.getTranslation()).getAngle();
                    targetPose = new Pose2d(
                            currentPose.getTranslation().interpolate(nearestReefCenter, 0.2),
                            currentPose.getRotation().interpolate(reefToLastTargetLocation, 0.2)
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
