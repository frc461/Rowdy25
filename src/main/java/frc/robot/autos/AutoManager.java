package frc.robot.autos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotStates;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.autos.routines.AutoTrigger;
import frc.robot.util.FieldUtil;
import frc.robot.util.MultipleChooser;
import org.json.simple.parser.ParseException;

public final class AutoManager {
    public enum StartPosition {
        FAR_LEFT(1),
        CENTER_LEFT(2),
        CENTER(3),
        CENTER_RIGHT(4),
        FAR_RIGHT(5);

        final int index;
        StartPosition(int index) {
            this.index = index;
        }
    }

    public StartPosition startPosition;
    public List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocations;

    private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
    private final MultipleChooser<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocationsChooser = new MultipleChooser<>();

    public AutoManager() {
        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);

        for (FieldUtil.Reef.ScoringLocation location : FieldUtil.Reef.ScoringLocation.values()) {
            for (FieldUtil.Reef.Level level : FieldUtil.Reef.Level.values()) {
                scoringLocationsChooser.addOption(location.name() + level.level, new Pair<>(location, level));
            }
        }
        SmartDashboard.putData("Scoring Locations", scoringLocationsChooser);
    }


    public Command getFinalAutoCommand(RobotStates robotStates) {
        startPosition = startPositionChooser.getSelected();
        scoringLocations = scoringLocationsChooser.getSelected();

        return generateAutoEventLooper(startPosition, scoringLocations, robotStates).cmd();
    }


    private AutoEventLooper generateAutoEventLooper(
            StartPosition startPosition,
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

    private String getMostEfficientCoralStation(Pose2d currentLocation, Pose2d nextLocation) {
        double station1TotalDistance =
                currentLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(0).pose2d.getTranslation())
                + nextLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(0).pose2d.getTranslation());
        double station2TotalDistance =
                currentLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(1).pose2d.getTranslation())
                + nextLocation.getTranslation().getDistance(FieldUtil.CoralStation.getCoralStationTags().get(1).pose2d.getTranslation());
        return station1TotalDistance < station2TotalDistance ? "1-station" : "2-station";
    }
}
