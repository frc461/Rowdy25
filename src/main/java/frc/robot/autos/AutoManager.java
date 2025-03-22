package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotStates;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.autos.routines.AutoTrigger;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import frc.robot.util.MultipleChooser;

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

        private static Pose2d getStartingPosition(StartPosition startPosition) {
            return switch (startPosition) {
                case DRIVER_FAR_RIGHT -> new Pose2d(7.152226027397259, 0.8170162671232879, Rotation2d.kPi);
                case DRIVER_CENTER_RIGHT -> new Pose2d(7.152226027397259, 2.439790239726027, Rotation2d.kPi);
                case DRIVER_CENTER -> new Pose2d(7.152226027397259, 4.04753852739726, Rotation2d.kPi);
                case DRIVER_CENTER_LEFT -> new Pose2d(7.152226027397259, 5.616003247853871, Rotation2d.kPi);
                case DRIVER_FAR_LEFT -> new Pose2d(7.152226027397259, 7.271, Rotation2d.kPi);
            };
        }
    }

    private StartPosition startPosition = null;
    private List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocations = null;
    private String coralStationOverride = null;
    private boolean push = false; // Whether to push alliance partner off first

    public AutoManager(RobotStates robotStates) {

        SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);
        startPositionChooser.onChange(startPosition -> {
            this.startPosition = startPosition;
            if (this.startPosition != null && this.scoringLocations != null && !this.scoringLocations.isEmpty()) {
                currentCommand = generateAutoEventLooper(robotStates).cmd();
            }
        });

        MultipleChooser<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocationsChooser = new MultipleChooser<>();
        for (FieldUtil.Reef.ScoringLocation location : FieldUtil.Reef.ScoringLocation.values()) {
            for (FieldUtil.Reef.Level level : FieldUtil.Reef.Level.values()) {
                scoringLocationsChooser.addOption(location.name() + level.level, new Pair<>(location, level));
            }
        }
        SmartDashboard.putData("Scoring Locations", scoringLocationsChooser);
        scoringLocationsChooser.onChange(scoringLocations -> {
            this.scoringLocations = scoringLocations;
            if (startPosition != null && this.scoringLocations != null && !this.scoringLocations.isEmpty()) {
                currentCommand = generateAutoEventLooper(robotStates).cmd();
            }
        });

        SendableChooser<String> coralStationOverrideChooser = new SendableChooser<>();
        coralStationOverrideChooser.addOption("Driver Left Coral Station", "station-1");
        coralStationOverrideChooser.addOption("Driver Right Coral Station", "station-2");
        SmartDashboard.putData("Coral Station Preference", coralStationOverrideChooser);
        coralStationOverrideChooser.onChange(coralStationOverride -> {
            this.coralStationOverride = coralStationOverride;
            if (startPosition != null && this.scoringLocations != null && !this.scoringLocations.isEmpty()) {
                currentCommand = generateAutoEventLooper(robotStates).cmd();
            }
        });

        SendableChooser<Boolean> pushChooser = new SendableChooser<>();
        pushChooser.addOption("Push", true);
        pushChooser.addOption("Don't Push", false);
        SmartDashboard.putData("Push Alliance Partner First", pushChooser);
        pushChooser.onChange(push -> {
            this.push = push;
            if (startPosition != null && this.scoringLocations != null && !this.scoringLocations.isEmpty()) {
                currentCommand = generateAutoEventLooper(robotStates).cmd();
            }
        });

        currentCommand = Commands.none();
    }

    public Command getFinalAutoCommand() {
        return currentCommand;
    }


    private AutoEventLooper generateAutoEventLooper(
            RobotStates robotStates
    ) {
        List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> currentScoringLocations = new ArrayList<>(this.scoringLocations);

        AutoEventLooper autoEventLooper = new AutoEventLooper("AutoEventLooper");
        List<AutoTrigger> triggersToBind = new ArrayList<>();

        triggersToBind.add(autoEventLooper.addTrigger(
                "start",
                () -> new InstantCommand(() -> robotStates.swerve.localizer.setPoses(getStartingPose(startPosition)))
                        .andThen(robotStates::setStowState)
        ));

        if (push) {
            triggersToBind.add(autoEventLooper.addTrigger(
                    "push",
                    robotStates.swerve::pushAlliancePartnerOut
            ));
        }

        Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> firstScoringLocation = currentScoringLocations.get(0);

        triggersToBind.add(autoEventLooper.addTrigger(
                this.startPosition.index + "," + firstScoringLocation.getFirst().name(),
                () -> robotStates.swerve.pathFindToScoringLocation(robotStates, firstScoringLocation.getFirst(), firstScoringLocation.getSecond())
        ));

        while (!currentScoringLocations.isEmpty()) {
            Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> currentScoringLocation = currentScoringLocations.remove(0);

            if (currentScoringLocations.isEmpty()) {
                break;
            }

            Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level> nextScoringLocation = currentScoringLocations.get(0);

            AtomicBoolean scoringNext = new AtomicBoolean(false);
            Command routineSegmentCommand = new WaitCommand(0.5)
                            .andThen(getPathFindingCommandToCoralStation(robotStates, currentScoringLocation.getFirst(), nextScoringLocation.getFirst()))
                            .andThen(new WaitUntilCommand(() -> robotStates.stowState.getAsBoolean() || robotStates.intake.coralEntered()))
                            .andThen(() -> scoringNext.set(true))
                            .andThen(robotStates.swerve.pathFindToScoringLocation(robotStates, nextScoringLocation.getFirst(), nextScoringLocation.getSecond()));

            Trigger dropsCoralTrigger = new Trigger(() -> scoringNext.get() && !robotStates.intake.barelyHasCoral() && !robotStates.atScoringLocation());
            dropsCoralTrigger.onTrue(new InstantCommand(routineSegmentCommand::cancel));
            autoEventLooper.observe(dropsCoralTrigger);

            triggersToBind.add(autoEventLooper.addTrigger(
                    currentScoringLocation.getFirst().name() + "," + nextScoringLocation.getFirst().name(),
                    () -> routineSegmentCommand
            ));
        }

        autoEventLooper.active().onTrue(triggersToBind.get(0).cmd());

        while (!triggersToBind.isEmpty()) {
            AutoTrigger currentTrigger = triggersToBind.remove(0);
            currentTrigger.interrupt().onTrue(currentTrigger.cmd());
            currentTrigger.done().onTrue(triggersToBind.isEmpty() ? Commands.none() : triggersToBind.get(0).cmd());
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

    private Pose2d getStartingPose(StartPosition startPosition) {
        Pose2d startingPoseBlue = StartPosition.getStartingPosition(startPosition);
        return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FlippingUtil.flipFieldPose(startingPoseBlue) : startingPoseBlue;
    }

    private Command getPathFindingCommandToCoralStation(RobotStates robotStates, FieldUtil.Reef.ScoringLocation current, FieldUtil.Reef.ScoringLocation next) {
        String coralStation = this.coralStationOverride == null
                ? getMostEfficientCoralStation(
                        FieldUtil.Reef.ScoringLocation.getPose(current),
                        FieldUtil.Reef.ScoringLocation.getPose(next)
                ) : this.coralStationOverride;

        if (coralStation.equals("station-1")) {
            return robotStates.swerve.pathFindToLeftCoralStation(robotStates);
        }
        return robotStates.swerve.pathFindToRightCoralStation(robotStates);
    }
}
