package frc.robot.autos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotStates;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.autos.routines.AutoTrigger;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import frc.robot.util.MultipleChooser;
import org.json.simple.parser.ParseException;

public final class AutoManager {
    private Command currentCommand;

    public enum StartPosition {
        DRIVER_FAR_RIGHT(1),
        DRIVER_CENTER_RIGHT(2),
        DRIVER_CENTER(3),
        DRIVER_CENTER_LEFT(4),
        DRIVER_FAR_LEFT(5);

        final int index;
        StartPosition(int index) {
            this.index = index;
        }
    }

    public StartPosition startPosition = null;
    public List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocations = null;

    private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
    private final MultipleChooser<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocationsChooser = new MultipleChooser<>();

    public AutoManager(RobotStates robotStates) {

        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);
        startPositionChooser.onChange(state -> {
            startPosition = startPositionChooser.getSelected();
            if (startPosition != null && scoringLocations != null) {
                currentCommand = generateAutoEventLooper(startPosition, scoringLocations, robotStates).cmd();
            }
        });

        for (FieldUtil.Reef.ScoringLocation location : FieldUtil.Reef.ScoringLocation.values()) {
            for (FieldUtil.Reef.Level level : FieldUtil.Reef.Level.values()) {
                scoringLocationsChooser.addOption(location.name() + level.level, new Pair<>(location, level));
            }
        }
        SmartDashboard.putData("Scoring Locations", scoringLocationsChooser);
        scoringLocationsChooser.onChange(states -> {
            scoringLocations = scoringLocationsChooser.getSelected();
            if (!(startPosition == null) && !(scoringLocations == null)) {
                currentCommand = generateAutoEventLooper(startPosition, scoringLocations, robotStates).cmd();
            }
        });

        currentCommand = Commands.none();
    }

    public Command getFinalAutoCommand() {
        return currentCommand;
    }


    private AutoEventLooper generateAutoEventLooper(
            StartPosition startPosition,
            List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocations,
            RobotStates robotStates
    ) {
        List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> currentScoringLocations = new ArrayList<>(scoringLocations);
        AutoEventLooper autoEventLooper = new AutoEventLooper("AutoEventLooper");

        List<AutoTrigger> triggers = new ArrayList<>();
        Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> firstScoringLocation = currentScoringLocations.get(0);
        String firstPath = startPosition.index + "," + firstScoringLocation.getFirst().name();

        triggers.add(autoEventLooper.addTrigger(
                firstPath,
                () -> {
                    try {
                        PathPlannerPath path = PathPlannerPath.fromPathFile(firstPath);

                        return new InstantCommand(() -> robotStates.setCurrentAutoLevel(firstScoringLocation.getSecond()))
                                .andThen(() -> robotStates.swerve.localizer.setPoses(getStartingPose(path)))
                                .andThen(AutoBuilder.followPath(path));
                    } catch (IOException | ParseException e) {
                        DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
                        return Commands.none();
                    }
                }
        ));

        triggers.add(autoEventLooper.addTrigger(
                "outtake",
                () -> new WaitUntilCommand(robotStates.atState)
                        .andThen(new WaitCommand(1.0))
                        .andThen(robotStates::toggleAutoLevelCoralState)
                        .andThen(new WaitUntilCommand(robotStates.stowState))
                        .andThen(new WaitCommand(0.5))
        ));

        while (!currentScoringLocations.isEmpty()) {
            Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> currentScoringLocation = currentScoringLocations.remove(0);

            if (currentScoringLocations.isEmpty()) {
                break;
            }

            Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> nextScoringLocation = currentScoringLocations.get(0);
            String nearestCoralStation = getMostEfficientCoralStation(
                    FieldUtil.Reef.ScoringLocation.getPose(currentScoringLocation.getFirst()),
                    FieldUtil.Reef.ScoringLocation.getPose(nextScoringLocation.getFirst())
            ); // TODO: IMPLEMENT A CORAL STATION PREFERENCE CHOOSER
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

            triggers.add(autoEventLooper.addTrigger("waitUntilHasObject", () -> 
                    new WaitUntilCommand(() -> robotStates.stowState.getAsBoolean() || robotStates.intake.hasCoral())));

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
                    () -> new WaitUntilCommand(robotStates.atState) // TODO SHOP: TEST IS AT AUTO STATE
                            .andThen(new WaitCommand(0.75)) // TODO SHOP: TUNE AND OPTIMIZE
                            .andThen(robotStates::toggleAutoLevelCoralState)
                            .andThen(new WaitUntilCommand(robotStates.stowState))
                            .andThen(new WaitCommand(0.5))
            ));
        }

        autoEventLooper.active().onTrue(triggers.get(0).cmd());

        while (!triggers.isEmpty()) {
            AutoTrigger currentTrigger = triggers.remove(0);
            currentTrigger.done().onTrue(triggers.isEmpty() ? Commands.none() : triggers.get(0).cmd());
        }

        return autoEventLooper;
    }

    private String getMostEfficientCoralStation(Pose2d currentLocation, Pose2d nextLocation) {
        List<FieldUtil.AprilTag> tags = FieldUtil.CoralStation.getCoralStationTags();
        double station1TotalDistance =
                currentLocation.getTranslation().getDistance(tags.get(0).pose2d.getTranslation())
                + nextLocation.getTranslation().getDistance(tags.get(0).pose2d.getTranslation());
        double station2TotalDistance =
                currentLocation.getTranslation().getDistance(tags.get(1).pose2d.getTranslation())
                + nextLocation.getTranslation().getDistance(tags.get(1).pose2d.getTranslation());
        return station1TotalDistance < station2TotalDistance ? "station-1" : "station-2";
    }

    private Pose2d getStartingPose(PathPlannerPath path) {
        Pose2d startingPoseBlue = path.getStartingHolonomicPose().orElse(Pose2d.kZero);
        return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FlippingUtil.flipFieldPose(startingPoseBlue) : startingPoseBlue;
    }
}
