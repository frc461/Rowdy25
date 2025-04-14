package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

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
import frc.robot.constants.RobotPoses;
import frc.robot.util.FieldUtil;
import frc.robot.util.MultipleChooser;

public final class AutoManager {
    private Command currentCommand;

    public enum StartPosition {
        CUSTOM(0),
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
                case CUSTOM -> new Pose2d();
                case DRIVER_FAR_RIGHT -> new Pose2d(7.152226027397259, 0.8170162671232879, Rotation2d.kPi);
                case DRIVER_CENTER_RIGHT -> new Pose2d(7.152226027397259, 2.439790239726027, Rotation2d.kPi);
                case DRIVER_CENTER -> new Pose2d(7.152226027397259, 4.04753852739726, Rotation2d.kPi);
                case DRIVER_CENTER_LEFT -> new Pose2d(7.152226027397259, 5.616003247853871, Rotation2d.kPi);
                case DRIVER_FAR_LEFT -> new Pose2d(7.152226027397259, 7.271, Rotation2d.kPi);
            };
        }
    }

    private StartPosition startPosition = null;
    private List<String> scoringOrAlgaeLocations = null;
    private String coralStationOverride = null;
    private boolean push = false; // Whether to push alliance partner off first
    private boolean groundIntake = false; // Whether or not to use ground intake states instead of coral station states

    public AutoManager(RobotStates robotStates) {

        SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);
        startPositionChooser.onChange(startPosition -> {
            this.startPosition = startPosition;
            if (this.startPosition != null && this.scoringOrAlgaeLocations != null && !this.scoringOrAlgaeLocations.isEmpty()) {
                currentCommand = generateAutoEventLooperCommand(robotStates);
            }
        });
        
        MultipleChooser<String> scoringOrAlgaeLocationsChooser = new MultipleChooser<>();
        for (FieldUtil.Reef.ScoringLocation location : FieldUtil.Reef.ScoringLocation.values()) {
            for (FieldUtil.Reef.Level level : FieldUtil.Reef.Level.values()) {
                scoringOrAlgaeLocationsChooser.addOption(location.name() + level.level, location.name() + level.level);
            }
        }
        for (FieldUtil.Reef.Side side : FieldUtil.Reef.Side.values()) {
            scoringOrAlgaeLocationsChooser.addOption(side.name(), side.name());
        }
        SmartDashboard.putData("Scoring or Algae Locations", scoringOrAlgaeLocationsChooser);
        scoringOrAlgaeLocationsChooser.onChange( scoringOrAlgaeLocations -> {
            this.scoringOrAlgaeLocations = scoringOrAlgaeLocations;
            if (startPosition != null && this.scoringOrAlgaeLocations != null && !this.scoringOrAlgaeLocations.isEmpty()) {
                currentCommand = generateAutoEventLooperCommand(robotStates);
            }
        });

        SendableChooser<String> coralStationOverrideChooser = new SendableChooser<>();
        coralStationOverrideChooser.addOption("Driver Left Coral Station", "station-1");
        coralStationOverrideChooser.addOption("Driver Right Coral Station", "station-2");
        SmartDashboard.putData("Coral Station Preference", coralStationOverrideChooser);
        coralStationOverrideChooser.onChange(coralStationOverride -> {
            this.coralStationOverride = coralStationOverride;
            if (startPosition != null && this.scoringOrAlgaeLocations != null && !this.scoringOrAlgaeLocations.isEmpty()) {
                currentCommand = generateAutoEventLooperCommand(robotStates);
            }
        });

        SendableChooser<Boolean> pushChooser = new SendableChooser<>();
        pushChooser.addOption("Push", true);
        pushChooser.addOption("Don't Push", false);
        SmartDashboard.putData("Push Alliance Partner First", pushChooser);
        pushChooser.onChange(push -> {
            this.push = push;
            if (startPosition != null && this.scoringOrAlgaeLocations != null && !this.scoringOrAlgaeLocations.isEmpty()) {
                currentCommand = generateAutoEventLooperCommand(robotStates);
            }
        });

        SendableChooser<Boolean> groundIntakeChooser = new SendableChooser<>();
        groundIntakeChooser.addOption("Ground Intake", true);
        groundIntakeChooser.addOption("Coral Station Intake", false);
        SmartDashboard.putData("Intake Type", groundIntakeChooser);
        groundIntakeChooser.onChange(groundIntake -> {
            this.groundIntake = groundIntake;
            if (startPosition != null && this.scoringOrAlgaeLocations != null && !this.scoringOrAlgaeLocations.isEmpty()) {
                currentCommand = generateAutoEventLooperCommand(robotStates);
            }
        });

        currentCommand = Commands.none();
    }

    public Command getFinalAutoCommand() {
        return currentCommand;
    }


    private Command generateAutoEventLooperCommand(
            RobotStates robotStates
    ) {
        List<String> currentScoringLocations = new ArrayList<>(this.scoringOrAlgaeLocations);

        AutoEventLooper autoEventLooper = new AutoEventLooper("AutoEventLooper");
        List<AutoTrigger> triggersToBind = new ArrayList<>();

        String firstScoringOrAlgaeLocation = currentScoringLocations.get(0);
        getScoringLocation(firstScoringOrAlgaeLocation).ifPresentOrElse(
                firstScoringLocation -> triggersToBind.add(autoEventLooper.addTrigger(
                        this.startPosition.index + "," + firstScoringOrAlgaeLocation,
                        () -> new InstantCommand(() -> robotStates.swerve.localizer.setPoses(getStartingPose(startPosition)))
                                .onlyIf(() -> startPosition.index != 0)
                                .andThen(robotStates::setStowState)
                                .andThen(new InstantCommand(robotStates::setL2L3L4StowState).onlyIf(() -> firstScoringLocation.getSecond() != FieldUtil.Reef.Level.L1))
                                .andThen(robotStates.swerve.pushAlliancePartnerOut()).onlyIf(() -> push)
                                .andThen(robotStates.swerve.pathFindToScoringLocation(robotStates, firstScoringLocation.getFirst(), firstScoringLocation.getSecond()))
                )),
                () -> getAlgaeLocation(firstScoringOrAlgaeLocation).ifPresent(
                        firstAlgaeLocation -> triggersToBind.add(autoEventLooper.addTrigger(
                                this.startPosition.index + "," + firstScoringOrAlgaeLocation,
                                () -> new InstantCommand(() -> robotStates.swerve.localizer.setPoses(getStartingPose(startPosition)))
                                        .onlyIf(() -> startPosition.index != 0)
                                        .andThen(robotStates::setStowState)
                                        .andThen(robotStates.swerve.pushAlliancePartnerOut()).onlyIf(() -> push)
                                        .andThen(robotStates.swerve.pathFindToAlgaeOnReef(robotStates, firstAlgaeLocation))
                                        .andThen(robotStates.swerve.pathFindToNet(robotStates))
                        ))
                )
        );

        while (!currentScoringLocations.isEmpty()) {
            String currentScoringOrAlgaeLocation = currentScoringLocations.remove(0);
            Pose2d currentScoringOrAlgaePose = getScoringLocation(currentScoringOrAlgaeLocation).map(
                    currentScoringLocation -> RobotPoses.Reef.getRobotPoseAtBranch(
                            robotStates.swerve.localizer.currentRobotScoringSetting,
                            currentScoringLocation.getFirst()
                    )
            ).orElseGet(
                    () -> getAlgaeLocation(currentScoringOrAlgaeLocation).map(
                            RobotPoses.Reef::getRobotPoseAtAlgaeReef
                    ).orElseGet(Pose2d::new)
            );

            if (currentScoringLocations.isEmpty()) {
                if (robotStates.swerve.localizer.onStartingLine()) {
                    triggersToBind.add(autoEventLooper.addTrigger(
                            currentScoringOrAlgaeLocation + "," + "away",
                            () -> Commands.waitSeconds(0.5)
                                    .andThen(robotStates.swerve.moveAwayFromStartingLine(robotStates))
                    ));
                }
                break;
            }

            String nextScoringOrAlgaeLocation = currentScoringLocations.get(0);

            AtomicBoolean scoringNextCoral = new AtomicBoolean(false);
            AtomicReference<AutoTrigger> scoreCoralTriggerToBind = new AtomicReference<>();

            AtomicBoolean scoringNextAlgae = new AtomicBoolean(false);

            getScoringLocation(nextScoringOrAlgaeLocation).ifPresentOrElse(
                    nextScoringLocation -> {
                        scoreCoralTriggerToBind.set(autoEventLooper.addTrigger(
                                currentScoringOrAlgaeLocation + "," + nextScoringOrAlgaeLocation,
                                () -> Commands.waitSeconds(0.5) // TODO SHOP: MINIMIZE THIS
                                        .andThen(groundIntake
                                                ? getPathFindingCommandToGroundIntakeCoral(robotStates, currentScoringOrAlgaePose, RobotPoses.Reef.getRobotPoseAtBranch(
                                                        robotStates.swerve.localizer.currentRobotScoringSetting,
                                                        nextScoringLocation.getFirst()
                                                )) : getPathFindingCommandToCoralStation(robotStates, currentScoringOrAlgaePose, RobotPoses.Reef.getRobotPoseAtBranch(
                                                        robotStates.swerve.localizer.currentRobotScoringSetting,
                                                        nextScoringLocation.getFirst()
                                                ))
                                        )
                                        .andThen(new WaitUntilCommand(() -> robotStates.stowState.getAsBoolean() || robotStates.intake.coralEntered()))
                                        .andThen(() -> scoringNextCoral.set(true))
                                        .andThen(robotStates.swerve.pathFindToScoringLocation(robotStates, nextScoringLocation.getFirst(), nextScoringLocation.getSecond()))
                                        .andThen(() -> scoringNextCoral.set(false))
                        ));
                        triggersToBind.add(scoreCoralTriggerToBind.get());
                    },
                    () -> getAlgaeLocation(nextScoringOrAlgaeLocation).ifPresent(
                            nextAlgaeLocation -> triggersToBind.add(autoEventLooper.addTrigger(
                                    currentScoringOrAlgaeLocation + "," + nextScoringOrAlgaeLocation,
                                    () -> Commands.waitSeconds(FieldUtil.Reef.Side.algaeIsHigh(nextAlgaeLocation) ? 0.5 : 1.0)
                                            .andThen(robotStates.swerve.pathFindToAlgaeOnReef(robotStates, nextAlgaeLocation))
                                            .andThen(() -> scoringNextAlgae.set(true))
                                            .andThen(robotStates.swerve.pathFindToNet(robotStates))
                                            .andThen(() -> scoringNextAlgae.set(false))
                                            .until(() -> scoringNextAlgae.get() && !robotStates.intake.hasAlgae() && !robotStates.atScoringLocation())
                            ))
            ));

            new Trigger(() -> scoringNextCoral.get() && !robotStates.intake.barelyHasCoral() && !robotStates.atScoringLocation() || robotStates.intake.coralStuck())
                    .onTrue(new InstantCommand(() -> {
                        scoringNextCoral.set(false);
                        if (scoreCoralTriggerToBind.get() != null) {
                            scoreCoralTriggerToBind.get().cmd().cancel();
                        }
                    }));
        }

        new Trigger(autoEventLooper.active()).onFalse(
                new InstantCommand(robotStates::setStowState)
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(robotStates.swerve.moveAwayFromStartingLine(robotStates))
                        .onlyIf(DriverStation::isAutonomousEnabled) // If net routine happens when auto is about to end, stow so robot can move away from starting line
        );

        autoEventLooper.active().onTrue(triggersToBind.get(0).cmd());

        while (!triggersToBind.isEmpty()) {
            AutoTrigger currentTrigger = triggersToBind.remove(0);
            currentTrigger.interrupt().onTrue(
                    new InstantCommand(robotStates.intake::setOuttakeState)
                            .andThen(robotStates::setStowState)
                            .andThen(Commands.waitSeconds(0.5))
                            .andThen(new ScheduleCommand(currentTrigger.cmd()))
            );
            currentTrigger.done().onTrue(triggersToBind.isEmpty() ? Commands.none() : triggersToBind.get(0).cmd());
        }

        return autoEventLooper.cmd(() -> DriverStation.isAutonomousEnabled() && DriverStation.getMatchTime() <= 2 && robotStates.swerve.localizer.onStartingLine());
    }

    private Optional<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> getScoringLocation(String scoringLocation) {
        for (FieldUtil.Reef.ScoringLocation location : FieldUtil.Reef.ScoringLocation.values()) {
            for (FieldUtil.Reef.Level level : FieldUtil.Reef.Level.values()) {
                if (scoringLocation.equals(location.name() + level.level)) {
                    return Optional.of(new Pair<>(location, level));
                }
            }
        }
        return Optional.empty();
    }

    private Optional<FieldUtil.Reef.Side> getAlgaeLocation(String algaeLocation) {
        for (FieldUtil.Reef.Side side : FieldUtil.Reef.Side.values()) {
            if (algaeLocation.equals(side.name())) {
                return Optional.of(side);
            }
        }
        return Optional.empty();
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

    private Command getPathFindingCommandToCoralStation(RobotStates robotStates, Pose2d current, Pose2d next) {
        String coralStation = this.coralStationOverride == null
                ? getMostEfficientCoralStation(current, next)
                : this.coralStationOverride;

        if (coralStation.equals("station-1")) {
            return robotStates.swerve.pathFindToLeftCoralStation(robotStates);
        }
        return robotStates.swerve.pathFindToRightCoralStation(robotStates);
    }

    private Command getPathFindingCommandToGroundIntakeCoral(RobotStates robotStates, Pose2d current, Pose2d next) {
        String coralStation = this.coralStationOverride == null
                ? getMostEfficientCoralStation(current, next)
                : this.coralStationOverride;

        if (coralStation.equals("station-1")) {
            return robotStates.swerve.pathFindToLeftCoralStationGroundIntakeCoral(robotStates);
        }
        return robotStates.swerve.pathFindToRightCoralStationGroundIntakeCoral(robotStates);
    }
}
