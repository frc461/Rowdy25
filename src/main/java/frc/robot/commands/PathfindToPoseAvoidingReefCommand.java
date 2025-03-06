package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.Pathfinder;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.EquationUtil;
import frc.robot.util.FieldUtil;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;

public class PathfindToPoseAvoidingReefCommand extends Command { // TODO: IMPLEMENT SPHERICAL MODEL TO AVOID REEF COLLISION
    private enum Sides {
        AB, CD, EF, GH, IJ, KL;

        private static Rotation2d getDegreeStart(Sides side) {
            return switch (side) {
                case AB -> FieldUtil.Reef.getReefCorners().get(0).getRotation();
                case CD -> FieldUtil.Reef.getReefCorners().get(1).getRotation();
                case EF -> FieldUtil.Reef.getReefCorners().get(2).getRotation();
                case GH -> FieldUtil.Reef.getReefCorners().get(3).getRotation();
                case IJ -> FieldUtil.Reef.getReefCorners().get(4).getRotation();
                case KL -> FieldUtil.Reef.getReefCorners().get(5).getRotation();
            };
        }

        private static Sides getSide(Pose2d pose) {
            Rotation2d reefCenterAngleToRobot = pose.getTranslation().minus(FieldUtil.Reef.getReefCenter()).getAngle();
            if (Pathfinder.inBetween(reefCenterAngleToRobot, getDegreeStart(AB), getDegreeStart(CD))) {
                return AB;
            } else if (Pathfinder.inBetween(reefCenterAngleToRobot, getDegreeStart(CD), getDegreeStart(EF))) {
                return CD;
            } else if (Pathfinder.inBetween(reefCenterAngleToRobot, getDegreeStart(EF), getDegreeStart(GH))) {
                return EF;
            } else if (Pathfinder.inBetween(reefCenterAngleToRobot, getDegreeStart(GH), getDegreeStart(IJ))) {
                return GH;
            } else if (Pathfinder.inBetween(reefCenterAngleToRobot, getDegreeStart(IJ), getDegreeStart(KL))) {
                return IJ;
            } else {
                return KL;
            }
        }
    }

    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private final Pose2d targetPose;
    private Pose2d temporaryTargetPose;
    private boolean xPosDone, yPosDone, yawDone, end;

    public PathfindToPoseAvoidingReefCommand( // TODO SHOP: TEST THIS IN REAL LIFE
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose
    ) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;

        yawController = new PIDController(
                Constants.SwerveConstants.ANGULAR_POSITION_P,
                0,
                Constants.SwerveConstants.ANGULAR_POSITION_D
        );
        yawController.enableContinuousInput(Constants.SwerveConstants.ANGULAR_MINIMUM_ANGLE, Constants.SwerveConstants.ANGULAR_MAXIMUM_ANGLE);

        this.elevatorHeight = elevatorHeight;

        this.targetPose = targetPose;
        temporaryTargetPose = new Pose2d();
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        temporaryTargetPose = new Pose2d();
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        temporaryTargetPose = getTemporaryTargetPose(currentPose);

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(MathUtil.clamp(
                                EquationUtil.linearOutput(temporaryTargetPose.getX() - currentPose.getX(), 1.0),
                                -Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()),
                                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
                        ))
                        .withVelocityY(MathUtil.clamp(
                                EquationUtil.linearOutput(temporaryTargetPose.getY() - currentPose.getY(), 1.0),
                                -Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()),
                                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
                        ))
                        .withRotationalRate(yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                temporaryTargetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble()))
        );

        xPosDone = Math.abs(currentPose.getX() - targetPose.getX())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yPosDone = Math.abs(currentPose.getY() - targetPose.getY())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yawDone = Math.abs(MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180))
                < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;

        if (xPosDone && yPosDone && yawDone) {
            swerve.forceStop();
            swerve.consistentHeading = currentPose.getRotation().getDegrees();
            end = true;
        }
    }

    private Pose2d getTemporaryTargetPose(Pose2d currentPose) {
        Rotation2d robotAngleToReefCenter = FieldUtil.Reef.getAngleFromReefCenter(currentPose);

        if (currentPose.getTranslation().getDistance(FieldUtil.Reef.getReefCenter()) < FieldUtil.Reef.REEF_APOTHEM + Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters)) {
            Translation2d targetTranslation = new Pose2d(currentPose.getTranslation(), robotAngleToReefCenter)
                    .plus(new Transform2d(
                            new Translation2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) * 2, 0),
                            Rotation2d.kZero
                    ))
                    .getTranslation();
            return new Pose2d(targetTranslation, currentPose.getRotation());
        } else if (robotAngleToReefCenter.plus(targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().unaryMinus()).getDegrees() > 90.0
                || sameSideAsTargetPose(currentPose)) {
            return targetPose;
        } else {
            Rotation2d reefCenterAngleToRobot = robotAngleToReefCenter.unaryMinus();
            Rotation2d reefCenterAngleToTargetPose = targetPose.getTranslation().minus(FieldUtil.Reef.getReefCenter()).getAngle();
            Rotation2d temporaryTargetAngle =
                    reefCenterAngleToRobot.rotateBy(Rotation2d.fromDegrees(Math.copySign(
                            5.0,
                            reefCenterAngleToTargetPose.minus(reefCenterAngleToRobot).getDegrees()
                    )));
            return new Pose2d(
                    new Pose2d(FieldUtil.Reef.getReefCenter(), temporaryTargetAngle).plus(new Transform2d(
                            new Translation2d(1.4, 0),
                            Rotation2d.kZero
                    )).getTranslation(),
                    robotAngleToReefCenter
            );
        }
    }

    private boolean sameSideAsTargetPose(Pose2d currentPose) {
        Pose2d frontLeft = currentPose.plus(new Transform2d(
                new Translation2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0),
                Rotation2d.kZero
        ));
        Pose2d frontRight = currentPose.plus(new Transform2d(
                new Translation2d(Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, -Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0),
                Rotation2d.kZero
        ));
        Pose2d backLeft = currentPose.plus(new Transform2d(
                new Translation2d(-Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0),
                Rotation2d.kZero
        ));
        Pose2d backRight = currentPose.plus(new Transform2d(
                new Translation2d(-Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0, -Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0),
                Rotation2d.kZero
        ));
        return sameSideAsPose(frontLeft, targetPose) && sameSideAsPose(frontRight, targetPose)
                && sameSideAsPose(backLeft, targetPose) && sameSideAsPose(backRight, targetPose);
    }

    private boolean sameSideAsPose(Pose2d one, Pose2d two) {
        return Sides.getSide(one) == Sides.getSide(two);
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
