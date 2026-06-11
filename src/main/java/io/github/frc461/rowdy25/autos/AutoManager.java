package io.github.frc461.rowdy25.autos;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

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
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.autos.routines.AutoEventLooper;
import io.github.frc461.rowdy25.autos.routines.AutoTrigger;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.MultipleChooser;

/**
 * Manages and dynamically generates autonomous routines based on user selections from the SmartDashboard (Shuffleboard/Elastic).
 * This class handles starting positions, sequences of scoring locations (coral and algae), paths, and general autonomous preferences.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class AutoManager {
    /** The currently compiled autonomous command based on the selected preferences. */
    private Command currentCommand;

    /** Enum representing the possible starting positions of the robot on the field (relative to the driver station). */
    public enum StartPosition {
        /** Custom or unspecified starting position. */
        CUSTOM(0),
        /** Starting position on the far right relative to the driver station. */
        DRIVER_FAR_RIGHT(1),
        /** Starting position on the center-right relative to the driver station. */
        DRIVER_CENTER_RIGHT(2),
        /** Starting position in the direct center relative to the driver station. */
        DRIVER_CENTER(3),
        /** Starting position on the center-left relative to the driver station. */
        DRIVER_CENTER_LEFT(4),
        /** Starting position on the far left relative to the driver station. */
        DRIVER_FAR_LEFT(5);

        /** The numerical index associated with the starting position. */
        final int index;

        /**
         * Constructs a StartPosition enum constant.
         *
         * @param index The numerical tracking index.
         */
        StartPosition(int index) {
            this.index = index;
        }

        /**
         * Retrieves the standard blue alliance starting pose for a given start position.
         *
         * @param startPosition The generalized start position.
         * @return A {@link Pose2d} representing the starting coordinates.
         */
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

    /** The selected starting position of the robot. */
    private StartPosition startPosition = null;

    /** The ordered list of selected scoring or algae locations to traverse during the auto routine. */
    private List<String> scoringOrAlgaeLocations = null;

    /** The user-specified override for the coral station to use, or null if it should be dynamically determined based on efficiency. */
    private String coralStationOverride = null;

    /** Indicates whether the robot should push a stalled alliance partner off the start line prior to the rest of the routine. */
    private boolean push = false;

    /** Indicates whether the robot should use ground intake states instead of the standard coral station intake states. */
    private boolean groundIntake = false;

    /**
     * Constructs an {@link AutoManager} and initializes SmartDashboard choosers for autonomous preferences.
     * Starts listening for user updates to regenerate the active autonomous routine execution paths dynamically.
     *
     * @param robotStates The central state manager of the robot.
     */
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

    /**
     * Retrieves the dynamically generated autonomous command based on the current SmartDashboard selections.
     *
     * @return The final autonomous {@link Command}.
     */
    public Command getFinalAutoCommand() {
        return currentCommand;
    }

    /**
     * Dynamically constructs an {@link AutoEventLooper} outlining the sequence of triggers and paths to execute the selected autonomous routine.
     *
     * @param robotStates The central state manager of the robot.
     * @return An initialized {@link AutoEventLooper} mapping out the autonomous phases.
     */
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
                                .andThen(new ConditionalCommand(
                                        new InstantCommand(robotStates::setStowState),
                                        new InstantCommand(robotStates::setL2L3L4StowState),
                                        () -> firstScoringLocation.getSecond() == FieldUtil.Reef.Level.L1
                                ))
                                .andThen(robotStates.swerve.pushAlliancePartnerOut().onlyIf(() -> push))
                                .andThen(robotStates.swerve.pathFindToScoringLocation(robotStates, firstScoringLocation.getFirst(), firstScoringLocation.getSecond()))
                )),
                () -> getAlgaeLocation(firstScoringOrAlgaeLocation).ifPresent(
                        firstAlgaeLocation -> triggersToBind.add(autoEventLooper.addTrigger(
                                this.startPosition.index + "," + firstScoringOrAlgaeLocation,
                                () -> new InstantCommand(() -> robotStates.swerve.localizer.setPoses(getStartingPose(startPosition)))
                                        .onlyIf(() -> startPosition.index != 0)
                                        .andThen(robotStates::setStowState)
                                        .andThen(robotStates.swerve.pushAlliancePartnerOut().onlyIf(() -> push))
                                        .andThen(robotStates.swerve.pathFindToAlgaeOnReef(robotStates, firstAlgaeLocation))
                                        .andThen(robotStates.swerve.pathFindToNet(robotStates, false))
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
                                            .andThen(robotStates.swerve.pathFindToNet(robotStates, false))
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

    /**
     * Parses a generalized scoring location string into a specific reef location and level.
     *
     * @param scoringLocation The string identifier of the scoring location.
     * @return An {@link Optional} containing a {@link Pair} representing the scoring side and level, or empty if the string doesn't correspond to a known configuration.
     */
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

    /**
     * Parses a generalized algae location string into a specific reef side.
     *
     * @param algaeLocation The string identifier of the algae location.
     * @return An {@link Optional} containing the corresponding reef {@link FieldUtil.Reef.Side}, or empty if not valid.
     */
    private Optional<FieldUtil.Reef.Side> getAlgaeLocation(String algaeLocation) {
        for (FieldUtil.Reef.Side side : FieldUtil.Reef.Side.values()) {
            if (algaeLocation.equals(side.name())) {
                return Optional.of(side);
            }
        }
        return Optional.empty();
    }

    /**
     * Determines the most efficient (closest total distance) coral station to travel to between the robot's current location and its next designated goal.
     *
     * @param currentLocation The starting point.
     * @param nextLocation The subsequent goal location after the station.
     * @return The string identifier of the chosen station: "station-1" (left) or "station-2" (right).
     */
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

    /**
     * Gets the starting field pose given a {@link PathPlannerPath} to start on, adapting for the current team alliance.
     *
     * @param path The PathPlanner path.
     * @return The alliance-adapted {@link Pose2d}.
     */
    private Pose2d getStartingPose(PathPlannerPath path) {
        Pose2d startingPoseBlue = path.getStartingHolonomicPose().orElse(Pose2d.kZero);
        return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FlippingUtil.flipFieldPose(startingPoseBlue) : startingPoseBlue;
    }

    /**
     * Gets the starting field pose associated with a user-selected starting index, adapting for the current team alliance.
     *
     * @param startPosition The user-selected start position enum.
     * @return The alliance-adapted {@link Pose2d}.
     */
    private Pose2d getStartingPose(StartPosition startPosition) {
        Pose2d startingPoseBlue = StartPosition.getStartingPosition(startPosition);
        return Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FlippingUtil.flipFieldPose(startingPoseBlue) : startingPoseBlue;
    }

    /**
     * Retrieves the command to pathfind to the appropriately determined coral station.
     *
     * @param robotStates The robot states wrapper.
     * @param current The current robot position.
     * @param next The ensuing robot position after station acquisition.
     * @return A {@link Command} initiating the station pathfinder routine.
     */
    private Command getPathFindingCommandToCoralStation(RobotStates robotStates, Pose2d current, Pose2d next) {
        String coralStation = this.coralStationOverride == null
                ? getMostEfficientCoralStation(current, next)
                : this.coralStationOverride;

        if (coralStation.equals("station-1")) {
            return robotStates.swerve.pathFindToLeftCoralStation(robotStates);
        }
        return robotStates.swerve.pathFindToRightCoralStation(robotStates);
    }

    /**
     * Retrieves the command to pathfind to a distance away from the appropriately determined coral station specifically configured for ground intake.
     *
     * <p>The pose to collect coral for ground intake differs as it is away from the coral station in order for more effective object detection in a wider field of view around the coral station area.</p>
     *
     * @param robotStates The robot states wrapper.
     * @param current The current robot position.
     * @param next The ensuing robot position after coral acquisition.
     * @return A {@link Command} initiating the ground-collection pathfinder routine.
     */
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
