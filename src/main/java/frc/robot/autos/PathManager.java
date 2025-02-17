package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotStates;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.autos.routines.AutoTrigger;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

public final class PathManager {
    public static AutoEventLooper generateAutoEventLooper(
            AutoChooser.StartPosition startPosition,
            List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocations,
            RobotStates robotStates
    ) {
        AutoEventLooper autoEventLooper = new AutoEventLooper("AutoEventLooper");

        List<AutoTrigger> triggers = new ArrayList<>();
        Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> firstScoringLocation = scoringLocations.get(0);
        String firstPath = startPosition.index + "," + firstScoringLocation.getFirst().name();

        triggers.add(autoEventLooper.addTrigger(
                firstPath,
                () -> {
                    try {
                        return new InstantCommand(() -> robotStates.setCurrentAutoLevel(firstScoringLocation.getSecond()))
                                .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile(firstPath)));
                    } catch (IOException | ParseException e) {
                        DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                    }
                }
        ));

        triggers.add(autoEventLooper.addTrigger(
                "outtake",
                () -> new InstantCommand(robotStates::toggleAutoLevelCoralState)
                        .andThen(new WaitUntilCommand(robotStates.stowState))
        ));

        while (!scoringLocations.isEmpty()) {
            Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> currentScoringLocation = scoringLocations.removeFirst();
            Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> nextScoringLocation = scoringLocations.getFirst();
            String nearestCoralStation = getMostEfficientCoralStation(
                    currentScoringLocation.getFirst().pose,
                    nextScoringLocation.getFirst().pose
            );
            String toCoralStationPath = currentScoringLocation.getFirst().name() + "," + nearestCoralStation;

            triggers.add(autoEventLooper.addTrigger(
                    toCoralStationPath,
                    () -> {
                        try {
                            return new InstantCommand(robotStates::resetCurrentAutoLevel)
                                    .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile(toCoralStationPath)));
                        } catch (IOException | ParseException e) {
                            DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                            return Commands.none();
                        }
                    }
            ));

            triggers.add(autoEventLooper.addTrigger("waitUntilHasObject", () -> new WaitUntilCommand(robotStates.stowState)));

            String fromCoralStationPath = nearestCoralStation + "," + nextScoringLocation.getFirst().name();

            triggers.add(autoEventLooper.addTrigger(
                    fromCoralStationPath,
                    () -> {
                        try {
                            return new InstantCommand(() -> robotStates.setCurrentAutoLevel(nextScoringLocation.getSecond()))
                                    .andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile(fromCoralStationPath)));
                        } catch (IOException | ParseException e) {
                            DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                            return Commands.none();
                        }
                    }
            ));

            triggers.add(autoEventLooper.addTrigger(
                    "outtake",
                    () -> new InstantCommand(robotStates::toggleAutoLevelCoralState)
                            .andThen(new WaitUntilCommand(robotStates.stowState))
            ));
        }

        autoEventLooper.active().onTrue(triggers.get(0).cmd());

        while (!triggers.isEmpty()) {
            triggers.remove(0).done().onTrue(triggers.isEmpty() ? Commands.none() : triggers.get(0).cmd());
        }

        return autoEventLooper;
    }

    private static String getMostEfficientCoralStation(Pose2d currentLocation, Pose2d nextLocation) {
        double station1TotalDistance =
                currentLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(0).pose2d.getTranslation())
                + nextLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(0).pose2d.getTranslation());
        double station2TotalDistance =
                currentLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(1).pose2d.getTranslation())
                + nextLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(1).pose2d.getTranslation());
        return station1TotalDistance < station2TotalDistance ? "1-station" : "2-station";
    }

    private static Command pathFindToPose(Pose2d targetPose, double goalEndVelocity) {
        return AutoBuilder.pathfindToPose(
                targetPose,
                Constants.AutoConstants.PATH_CONSTRAINTS,
                goalEndVelocity
        );
    }

    private static Command pathFindToPose(Pose2d targetPose) {
        return pathFindToPose(targetPose, 0.0);
    }

    public static Command pathFindToNearestAlgaeScoringLocation(Pose2d currentPose) {
        Pose2d nearestAlgaeScoringPose = FieldUtil.AlgaeScoring.getNearestAlgaeScoringTagPose(currentPose);
        return PathManager.pathFindToClosePose(
                new Pose2d(
                        nearestAlgaeScoringPose.getTranslation(),
                        nearestAlgaeScoringPose.getRotation().rotateBy(Rotation2d.kPi)
                ),
                Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO,
                1.0
        );
    }

    public static Command pathFindToNearestCoralScoringLocation(Pose2d currentPose) {
        Pose2d nearestCoralScoringPose = FieldUtil.Reef.getNearestBranchPose(currentPose);
        return PathManager.pathFindToClosePose(
                new Pose2d(
                        nearestCoralScoringPose.getTranslation(),
                        nearestCoralScoringPose.getRotation().rotateBy(Rotation2d.kPi)
                ),
                Constants.AutoConstants.DISTANCE_TOLERANCE_TO_DRIVE_INTO,
                1.0
        );
    }

    public static Command pathFindToClosePose(
            Pose2d targetPose,
            double distance,
            double goalEndVelocity
    ) {
        return pathFindToPose(
                new Pose2d(
                        targetPose.getTranslation().minus(new Translation2d(distance, targetPose.getRotation())),
                        targetPose.getRotation()
                ),
                goalEndVelocity
        );
    }

    public static Command pathFindToClosePose(
            Pose2d currentPose,
            Pose2d targetPose,
            double distance
    ) {
        return pathFindToClosePose(
                currentPose,
                targetPose,
                Rotation2d.fromDegrees(-180),
                Rotation2d.fromDegrees(180),
                distance
        );
    }

    public static Command pathFindToClosePose(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerThreshold,
            Rotation2d upperThreshold,
            double distance
    ) {
        return pathFindToClosePose(
                currentPose,
                targetPose,
                lowerThreshold,
                upperThreshold,
                distance,
                0.0
        );
    }

    public static Command pathFindToClosePose(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerThreshold,
            Rotation2d upperThreshold,
            double distance,
            double goalEndVelocity
    ) {
        if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < distance) {
            return Commands.none();
        }
        return pathFindToPose(
                calculateClosePoseWithAngleScopeAndRadius(
                        currentPose,
                        targetPose,
                        lowerThreshold,
                        upperThreshold,
                        distance
                ),
                goalEndVelocity
        );
    }

    private static Pose2d calculateClosePoseWithAngleScopeAndRadius(
            Pose2d currentPose,
            Pose2d targetPose,
            Rotation2d lowerAngleThreshold,
            Rotation2d upperAngleThreshold,
            double distance
    ) {
        Rotation2d distAngle = currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

        if (inBetween(distAngle, lowerAngleThreshold, upperAngleThreshold)) {
            return new Pose2d(
                    targetPose.getTranslation().plus(new Translation2d(distance, distAngle)),
                    distAngle.rotateBy(Rotation2d.kPi)
            );
        }
        return currentPose.nearest(List.of(
                new Pose2d(
                        targetPose.getTranslation().plus(new Translation2d(distance, lowerAngleThreshold)),
                        lowerAngleThreshold.rotateBy(Rotation2d.kPi)
                ),
                new Pose2d(
                        targetPose.getTranslation().plus(new Translation2d(distance, upperAngleThreshold)),
                        upperAngleThreshold.rotateBy(Rotation2d.kPi)
                )
        ));
    }

    private static boolean inBetween(Rotation2d angle, Rotation2d lowerAngleThreshold, Rotation2d upperAngleThreshold) {
        if (lowerAngleThreshold.getDegrees() > upperAngleThreshold.getDegrees()) {
            return angle.getDegrees() > lowerAngleThreshold.getDegrees() || angle.getDegrees() < upperAngleThreshold.getDegrees();
        }
        return angle.getDegrees() > lowerAngleThreshold.getDegrees() && angle.getDegrees() < upperAngleThreshold.getDegrees();
    }
}
