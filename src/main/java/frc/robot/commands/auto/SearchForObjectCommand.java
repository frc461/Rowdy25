package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.FieldUtil;
import frc.robot.util.VisionUtil;

public class SearchForObjectCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController errorController;
    private Translation2d targetTranslation;
    private double searchAngle;
    private double xVel; // for smooth transition after path interruption
    private double yVel;
    private double rotVel;
    private int transitionPoll;
    private double transitionMultiplier;
    private boolean translationComplete;
    private boolean end;

    public SearchForObjectCommand(Swerve swerve, SwerveRequest.FieldCentric fieldCentric) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        errorController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        errorController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        targetTranslation = new Translation2d(
                8.275 + 0.5 * (Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? 1 : (-1)),
                upperHalf() ? 0.5 : FieldUtil.FIELD_WIDTH - 0.5
        );
        searchAngle = Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red
                ? upperHalf()
                        ? Constants.AutoConstants.NOTE_SEARCH_DEGREE_SLANT
                        : -Constants.AutoConstants.NOTE_SEARCH_DEGREE_SLANT
                : upperHalf()
                        ? 180.0 - Constants.AutoConstants.NOTE_SEARCH_DEGREE_SLANT
                        : -180.0 + Constants.AutoConstants.NOTE_SEARCH_DEGREE_SLANT;

        ChassisSpeeds initial = swerve.getState().Speeds;
        Rotation2d rotationalOffset = swerve.localizer.getStrategyPose().getRotation();
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
        double currentYaw = swerve.localizer.getStrategyPose().getRotation().getDegrees();
        Translation2d currentTranslation = swerve.localizer.getStrategyPose().getTranslation();
        double currentX = currentTranslation.getX();
        double currentY = currentTranslation.getY();

        // TODO TEST SMOOTHNESS
        xVel *= 0.9;
        yVel *= 0.9;
        rotVel *= 0.9;
        System.out.println("xVel: " + xVel + ", yVel: " + yVel + ", rotVel: " + rotVel);
        transitionPoll++;
        transitionMultiplier = 1 - Math.pow(0.9, transitionPoll);

        if (!translationComplete) {
            double yError = Math.abs(targetTranslation.getY() - currentY);

            swerve.setControl(
                    fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(errorController.calculate(
                                    currentX,
                                    targetTranslation.getX()
                            ) * Constants.MAX_VEL * transitionMultiplier + xVel)
                            .withVelocityY(Constants.SwerveConstants.PATH_MANUAL_TRANSLATION_CONTROLLER.apply(yError)
                                    * (searchAngle > 0 ? -1 : 1) * transitionMultiplier + yVel)
                            .withRotationalRate(errorController.calculate(
                                    currentYaw,
                                    searchAngle
                            ) * transitionMultiplier + rotVel)
            );
            if (yError < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT) {
                translationComplete = true;
                end = true;
            }
        } else {
            swerve.forceStop();
            end = true;
        }

        if (VisionUtil.Photon.Color.hasTargets()) {
            end = true;
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }

    public boolean upperHalf() {
        return swerve.localizer.getStrategyPose().getY() > FieldUtil.FIELD_WIDTH / 2;
    }
}
