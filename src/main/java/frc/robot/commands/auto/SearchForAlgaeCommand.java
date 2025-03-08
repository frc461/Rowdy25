package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.EquationUtil;
import frc.robot.util.FieldUtil;
import frc.robot.util.vision.PhotonUtil;

public class SearchForAlgaeCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController errorController;
    private Pose2d currentPose;
    private Translation2d targetTranslation;
    private double searchAngle;
    private double xVel; // for smooth transition after path interruption
    private double yVel;
    private double rotVel;
    private int transitionPoll;
    private double transitionMultiplier;
    private boolean translationComplete;
    private boolean end;

    // TODO: REVAMP TO FOLLOW A PATH TO THE NEAREST CORAL STATION THEN TO THE OTHER, THEN STOP IF EITHER A CORAL IS FOUND OR PATH IS COMPLETED
    public SearchForAlgaeCommand(Swerve swerve, SwerveRequest.FieldCentric fieldCentric) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        errorController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        errorController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        currentPose = swerve.localizer.getStrategyPose();

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        currentPose = swerve.localizer.getStrategyPose();

        targetTranslation = new Translation2d(
                FieldUtil.FIELD_LENGTH / 2 + 0.5 * (Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? 1 : (-1)),
                upperHalf() ? 0.5 : FieldUtil.FIELD_WIDTH - 0.5
        );

        searchAngle = Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                ? upperHalf()
                        ? Constants.AutoConstants.OBJECT_SEARCH_DEGREE_SLANT
                        : -Constants.AutoConstants.OBJECT_SEARCH_DEGREE_SLANT
                : upperHalf()
                        ? 180.0 - Constants.AutoConstants.OBJECT_SEARCH_DEGREE_SLANT
                        : -180.0 + Constants.AutoConstants.OBJECT_SEARCH_DEGREE_SLANT;

        ChassisSpeeds initial = swerve.getState().Speeds;
        Rotation2d rotationalOffset = currentPose.getRotation();
        Translation2d translationVel = new Translation2d(initial.vxMetersPerSecond, initial.vyMetersPerSecond).rotateBy(rotationalOffset.unaryMinus());

        xVel = translationVel.getX();
        yVel = translationVel.getY();
        rotVel = initial.omegaRadiansPerSecond;
        transitionPoll = 0;
        transitionMultiplier = 0;

        translationComplete = false;
        end = false;
    }

    @Override
    public void execute() {
        currentPose = swerve.localizer.getStrategyPose();
        double currentYaw = currentPose.getRotation().getDegrees();
        Translation2d currentTranslation = currentPose.getTranslation();
        double currentX = currentTranslation.getX();
        double currentY = currentTranslation.getY();

        // TODO: TEST SMOOTHNESS
        xVel *= 0.9;
        yVel *= 0.9;
        rotVel *= 0.9;
        transitionPoll++;
        transitionMultiplier = 1 - Math.pow(0.9, transitionPoll);

        if (!translationComplete) {
            double xError = targetTranslation.getX() - currentX;
            double yError = targetTranslation.getY() - currentY;
            swerve.setControl(
                    fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                            .withVelocityX(EquationUtil.expOutput(Math.abs(xError), 4.0, 0.8, 6)
                                    * (xError < 0 ? -1 : 1) * transitionMultiplier + xVel)
                            .withVelocityY(EquationUtil.expOutput(Math.abs(yError), 4.0, 0.8, 6)
                                    * (yError < 0 ? -1 : 1) * transitionMultiplier + yVel)
                            .withRotationalRate(errorController.calculate(
                                    currentYaw,
                                    searchAngle
                            ) * transitionMultiplier + rotVel)
            );
            if (Math.abs(yError) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT) {
                translationComplete = true;
                end = true;
            }
        } else {
            swerve.forceStop();
            end = true;
        }

        if (PhotonUtil.Color.hasAlgaeTargets()) {
            end = true;
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }

    public boolean upperHalf() {
        return currentPose.getY() > FieldUtil.FIELD_WIDTH / 2;
    }
}
