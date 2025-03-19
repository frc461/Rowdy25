package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.Pathfinder;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.EquationUtil;
import frc.robot.util.FieldUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;

public class PathfindToPoseAvoidingReefCommand extends Command {
    private enum Side {
        AB, CD, EF, GH, IJ, KL;

        private static Pose2d getLeftVertexPose(Side side) {
            return switch (side) {
                case AB -> FieldUtil.Reef.getReefCorners().get(0);
                case CD -> FieldUtil.Reef.getReefCorners().get(1);
                case EF -> FieldUtil.Reef.getReefCorners().get(2);
                case GH -> FieldUtil.Reef.getReefCorners().get(3);
                case IJ -> FieldUtil.Reef.getReefCorners().get(4);
                case KL -> FieldUtil.Reef.getReefCorners().get(5);
            };
        }
    }

    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController yawController;
    private final DoubleSupplier elevatorHeight;
    private final Pose2d targetPose;
    private final double maxVelocity;
    private Pose2d smoothTemporaryTargetPose;
    private double xVel, yVel, rotVel, xOrigVel, yOrigVel, rotOrigVel, transitionPoll;
    private boolean xPosDone, yPosDone, yawDone, end;

    public PathfindToPoseAvoidingReefCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose
    ) {
        this(swerve, fieldCentric, elevatorHeight, targetPose, Constants.MAX_VEL);
    }

    public PathfindToPoseAvoidingReefCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            DoubleSupplier elevatorHeight,
            Pose2d targetPose,
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

        this.elevatorHeight = elevatorHeight;

        this.targetPose = targetPose;
        this.maxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_VEL);

        smoothTemporaryTargetPose = null;
        xVel = 0;
        yVel = 0;
        rotVel = 0;
        xOrigVel = 0;
        yOrigVel = 0;
        rotOrigVel = 0;
        transitionPoll = 0;
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        smoothTemporaryTargetPose = null;
        ChassisSpeeds chassisSpeeds = swerve.getState().Speeds;
        Translation2d fieldRelativeTranslation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(swerve.localizer.getStrategyPose().getRotation().unaryMinus());
        xVel = fieldRelativeTranslation.getX();
        yVel = fieldRelativeTranslation.getY();
        rotVel = chassisSpeeds.omegaRadiansPerSecond;
        xOrigVel = fieldRelativeTranslation.getX();
        yOrigVel = fieldRelativeTranslation.getY();
        rotOrigVel = chassisSpeeds.omegaRadiansPerSecond;
        transitionPoll = 0;
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        smoothTemporaryTargetPose = getSmoothTargetPose(getTemporaryTargetPose(currentPose));
        swerve.localizer.setCurrentTemporaryTargetPose(smoothTemporaryTargetPose);
        double safeMaxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()));

        double velocity = MathUtil.clamp(
                EquationUtil.expOutput(
                        smoothTemporaryTargetPose.getTranslation().getDistance(currentPose.getTranslation()),
                        safeMaxVelocity,
                        1.0 / 10.0 * Math.pow(safeMaxVelocity, 0.5) * Math.log(safeMaxVelocity * Math.exp(2.5) + safeMaxVelocity - 1), // TODO SHOP: TEST THESE VALUES
                        10.0 / Math.pow(safeMaxVelocity, 0.7)
                ),
                0,
                Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble())
        );

        double velocityHeadingRadians = Math.atan2(smoothTemporaryTargetPose.getY() - currentPose.getY(), smoothTemporaryTargetPose.getX() - currentPose.getX());

        updateVelocities(
                Math.cos(velocityHeadingRadians) * velocity,
                Math.sin(velocityHeadingRadians) * velocity,
                yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                smoothTemporaryTargetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble())
        );

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withDeadband(0.0)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(xVel)
                        .withVelocityY(yVel)
                        .withRotationalRate(rotVel)
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

    private void updateVelocities(double xTargetVel, double yTargetVel, double rotTargetVel) {
        transitionPoll++;
        xVel = Math.pow(0.9, transitionPoll) * xOrigVel + (1 - Math.pow(0.9, transitionPoll)) * xTargetVel;
        yVel = Math.pow(0.9, transitionPoll) * yOrigVel + (1 - Math.pow(0.9, transitionPoll)) * yTargetVel;
        rotVel = Math.pow(0.9, transitionPoll) * rotOrigVel + (1 - Math.pow(0.9, transitionPoll)) * rotTargetVel;
    }

    private Pose2d getSmoothTargetPose(Pose2d temporaryPose) {
        if (smoothTemporaryTargetPose == null) {
            return temporaryPose;
        }

        if (smoothTemporaryTargetPose.getTranslation().getDistance(temporaryPose.getTranslation()) > 0.11) {
            Rotation2d headingToTemporaryPose = temporaryPose.getTranslation().minus(smoothTemporaryTargetPose.getTranslation()).getAngle();
            return new Pose2d(
                    new Pose2d(smoothTemporaryTargetPose.getTranslation(), headingToTemporaryPose)
                            .plus(new Transform2d(0.11, 0, Rotation2d.kZero))
                            .getTranslation(),
                    smoothTemporaryTargetPose.getRotation().interpolate(temporaryPose.getRotation(), 0.25)
            );
        }

        return temporaryPose;
    }

    private Pose2d getTemporaryTargetPose(Pose2d currentPose) {
        Rotation2d reefCenterAngleToRobot = FieldUtil.Reef.getAngleFromReefCenter(currentPose);

        if (sameSideAsTargetPose(currentPose)) {
            return targetPose;
        } else if (currentPose.getTranslation().getDistance(FieldUtil.Reef.getReefCenter()) < FieldUtil.Reef.REEF_APOTHEM + Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 1.3) {
            Translation2d targetTranslation = new Pose2d(FieldUtil.Reef.getReefCenter(), FieldUtil.Reef.getNearestReefTagPose(currentPose).getRotation())
                    .plus(new Transform2d(3.0, 0, Rotation2d.kZero))
                    .getTranslation();
            return new Pose2d(targetTranslation, currentPose.getRotation());
        } else {
            Rotation2d reefCenterAngleToTargetPose = targetPose.getTranslation().minus(FieldUtil.Reef.getReefCenter()).getAngle();
            Rotation2d temporaryTargetAngle =
                    reefCenterAngleToRobot.rotateBy(Rotation2d.fromDegrees(Math.copySign(
                            45.0,
                            reefCenterAngleToTargetPose.minus(reefCenterAngleToRobot).getDegrees()
                    )));
            return new Pose2d(
                    new Pose2d(FieldUtil.Reef.getReefCenter(), temporaryTargetAngle)
                            .plus(new Transform2d(3.0, 0, Rotation2d.kZero))
                            .getTranslation(),
                    currentPose.getRotation().interpolate(targetPose.getRotation(), 0.25)
            );
        }
    }

    private boolean sameSideAsTargetPose(Pose2d currentPose) {
        List<Pose2d> robotCorners = List.of(
                currentPose.plus(new Transform2d(
                        Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                        Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                        Rotation2d.kZero
                )),
                currentPose.plus(new Transform2d(
                        Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                        -Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                        Rotation2d.kZero
                )),
                currentPose.plus(new Transform2d(
                        -Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                        Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                        Rotation2d.kZero
                )),
                currentPose.plus(new Transform2d(
                        -Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2.0,
                        -Constants.ROBOT_WIDTH_WITH_BUMPERS.in(Meters) / 2.0,
                        Rotation2d.kZero
                ))
        );

        List<Rotation2d> anglesToEachVertex = new ArrayList<>();
        List<Double> distancesToEachVertex = new ArrayList<>();

        for (Side side : Side.values()) {
            anglesToEachVertex.addAll(robotCorners.stream()
                    .map(corner -> Side.getLeftVertexPose(side).getTranslation().minus(corner.getTranslation()).getAngle())
                    .toList());
            distancesToEachVertex.addAll(robotCorners.stream()
                    .map(corner -> Side.getLeftVertexPose(side).getTranslation().getDistance(corner.getTranslation()))
                    .toList());
        }

        Pair<Rotation2d, Rotation2d> anglesToVerticesBounds = getBound(anglesToEachVertex);
        double lowestDistance = distancesToEachVertex.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);

        return !Pathfinder.inBetween(
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
                anglesToVerticesBounds.getFirst(),
                anglesToVerticesBounds.getSecond()
        ) && !Pathfinder.inBetween(
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
                anglesToVerticesBounds.getFirst().minus(Rotation2d.fromDegrees(15.0)),
                anglesToVerticesBounds.getSecond().plus(Rotation2d.fromDegrees(15.0))
        ) || targetPose.getTranslation().getDistance(currentPose.getTranslation()) < lowestDistance;
    }

    private Pair<Rotation2d, Rotation2d> getBound(List<Rotation2d> angles) {
        Rotation2d min = angles.get(0);
        Rotation2d max = angles.get(0);
        for (Rotation2d angle : angles) {
            if (!Pathfinder.inBetween(angle, min, max)) {
                Rotation2d divider = min.interpolate(max, 0.5).rotateBy(Rotation2d.kPi);
                if (Pathfinder.inBetween(angle, divider, min)) {
                    min = angle;
                } else {
                    max = angle;
                }
            }
        }
        return new Pair<>(min, max);
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
