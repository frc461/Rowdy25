package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
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

public class PathfindToPoseAvoidingReefCommand extends Command { // TODO: ORGANIZE INTO DRIVE COMMANDS SUBDIRECTORY
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
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();

        updateSmoothTargetPose(getTemporaryTargetPose(currentPose));

        smoothTemporaryTargetPose = null;
        xPosDone = false;
        yPosDone = false;
        yawDone = false;
        end = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.localizer.getStrategyPose();
        updateSmoothTargetPose(getTemporaryTargetPose(currentPose));
        swerve.localizer.setCurrentTemporaryTargetPose(smoothTemporaryTargetPose);
        double safeMaxVelocity = MathUtil.clamp(maxVelocity, 0, Constants.MAX_CONTROLLED_VEL.apply(elevatorHeight.getAsDouble()));

        double velocity = Math.max(
                EquationUtil.expOutput(
                        smoothTemporaryTargetPose.getTranslation().getDistance(currentPose.getTranslation()),
                        2,
                        2 / 7.0,
                        15 / 2.0
                ),
                Math.min(EquationUtil.linearOutput(smoothTemporaryTargetPose.getTranslation().getDistance(currentPose.getTranslation()), 10, -10), safeMaxVelocity) // TODO SHOP: TEST THIS OFFSET
        );

        double velocityHeadingRadians = smoothTemporaryTargetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().getRadians();

        swerve.setControl(
                fieldCentric.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withDeadband(0.0)
                        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                        .withVelocityX(Math.cos(velocityHeadingRadians) * velocity)
                        .withVelocityY(Math.sin(velocityHeadingRadians) * velocity)
                        .withRotationalRate(yawController.calculate(
                                currentPose.getRotation().getDegrees(),
                                smoothTemporaryTargetPose.getRotation().getDegrees()
                        ) * Constants.MAX_CONTROLLED_ANGULAR_VEL.apply(elevatorHeight.getAsDouble()))
        );

        xPosDone = Math.abs(currentPose.getX() - targetPose.getX())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yPosDone = Math.abs(currentPose.getY() - targetPose.getY())
                < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
        yawDone = Math.abs(MathUtil.inputModulus(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees(), -180, 180))
                < Constants.AutoConstants.DEGREE_TOLERANCE_TO_ACCEPT;

        if (xPosDone && yPosDone && yawDone) {
            end = true;
        }
    }

    private void updateSmoothTargetPose(Pose2d temporaryPose) {
        if (smoothTemporaryTargetPose == null) {
            smoothTemporaryTargetPose = temporaryPose;
        }

        if (smoothTemporaryTargetPose.getTranslation().getDistance(temporaryPose.getTranslation()) > 0.11) {
            Rotation2d headingToTemporaryPose = temporaryPose.getTranslation().minus(smoothTemporaryTargetPose.getTranslation()).getAngle();
            smoothTemporaryTargetPose = new Pose2d(
                    new Pose2d(smoothTemporaryTargetPose.getTranslation(), headingToTemporaryPose)
                            .plus(new Transform2d(0.11, 0, Rotation2d.kZero))
                            .getTranslation(),
                    smoothTemporaryTargetPose.getRotation().interpolate(temporaryPose.getRotation(), 0.25)
            );
        } else {
            smoothTemporaryTargetPose = temporaryPose;
        }
    }

    private Pose2d getTemporaryTargetPose(Pose2d currentPose) {
        Rotation2d reefCenterAngleToRobot = FieldUtil.Reef.getAngleFromReefCenter(currentPose);

        // TODO SHOP: TEST THIS
        if (sameSideAsTargetPose(currentPose)) {
            return targetPose;
        } else if (currentPose.getTranslation().getDistance(FieldUtil.Reef.getReefCenter()) < FieldUtil.Reef.REEF_APOTHEM + Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 1.3) {
            Translation2d targetTranslation = new Pose2d(FieldUtil.Reef.getReefCenter(), FieldUtil.Reef.getNearestReefTagPose(currentPose).getRotation())
                    .plus(new Transform2d(2.0, 0, Rotation2d.kZero))
                    .getTranslation();
            return new Pose2d(targetTranslation, currentPose.getRotation());
        } else {
            Rotation2d reefCenterAngleToTargetPose = targetPose.getTranslation().minus(FieldUtil.Reef.getReefCenter()).getAngle();
            Rotation2d temporaryTangentAngle =
                    reefCenterAngleToRobot.rotateBy(Rotation2d.fromDegrees(Math.copySign(
                            90.0,
                            reefCenterAngleToTargetPose.minus(reefCenterAngleToRobot).getDegrees()
                    )));
            return new Pose2d(
                    new Pose2d(
                            new Pose2d(FieldUtil.Reef.getReefCenter(), reefCenterAngleToRobot)
                                    .plus(new Transform2d(
                                            2.0,
                                            0,
                                            Rotation2d.kZero)
                                    ).getTranslation(),
                            temporaryTangentAngle
                    ).plus(new Transform2d(
                            1.5,
                            0,
                            Rotation2d.kZero
                    )).getTranslation(),
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
        double lowestDistanceToReefCorner = distancesToEachVertex.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);

        return !Pathfinder.inBetween(
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
                anglesToVerticesBounds.getFirst(),
                anglesToVerticesBounds.getSecond()
        ) && !Pathfinder.inBetween(
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle(),
                anglesToVerticesBounds.getFirst().minus(Rotation2d.fromDegrees(7.5)),
                anglesToVerticesBounds.getSecond().plus(Rotation2d.fromDegrees(7.5))
        ) || targetPose.getTranslation().getDistance(currentPose.getTranslation()) < lowestDistanceToReefCorner;
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
    public void end(boolean interrupted) {
        swerve.forceStop();
        swerve.consistentHeading = swerve.localizer.getStrategyPose().getRotation().getDegrees();
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}
