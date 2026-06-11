package io.github.frc461.rowdy25.subsystems.localizer;

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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.frc461.rowdy25.RobotStates;
import io.github.frc461.rowdy25.constants.Constants;
import io.github.frc461.rowdy25.constants.RobotPoses;
import io.github.frc461.rowdy25.subsystems.drivetrain.Swerve;
import io.github.frc461.rowdy25.util.EstimatedRobotPose;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.vision.LimelightUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;
import io.github.frc461.rowdy25.util.vision.QuestNavUtil;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

/**
 * Subsystem extension that manages robot localization and field-relative pose estimation.
 * <p>
 * Fuses odometry with PhotonVision and Limelight tag-based pose estimates using a
 * {@link SwerveDrivePoseEstimator}, supports QuestNav absolute positioning, computes
 * nearest reef/coral-station/algae target poses, and tracks scoring locations and
 * transition distances for autonomous commands.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 */
public class Localizer {
    /** Localization strategy enumeration for selecting between pose estimator and QuestNav. */
    private enum LocalizationStrategy {
        /** Use WPILib SwerveDrivePoseEstimator with vision fusion. */
        POSE_ESTIMATOR,
        /** Use QuestNav absolute positioning. */
        QUEST_NAV
    }

    /** The swerve drivetrain subsystem. */
    private final Swerve swerve;

    /** Proximity sensor for detecting nearby objects (not yet fully tested). */
    private final DigitalInput proximitySensor = new DigitalInput(Constants.VisionConstants.PROXIMITY_SENSOR_DIO_PORT); // TODO SHOP: TEST MORE

    /** Telemetry publisher for localization state. */
    private final LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry(this);

    /** SmartDashboard chooser for selecting the localization strategy. */
    private final SendableChooser<LocalizationStrategy> localizationChooser = new SendableChooser<>();

    /** The WPILib swerve drive pose estimator. */
    private final SwerveDrivePoseEstimator poseEstimator;

    /** The current pose estimation strategy. Defaults to POSE_ESTIMATOR. */
    private LocalizationStrategy strategy = LocalizationStrategy.POSE_ESTIMATOR;

    /** The current temporary target pose for pathfinding waypoints. */
    private Pose2d currentTemporaryTargetPose = new Pose2d();

    /** Whether QuestNav has calibrated once when the robot is near a tag. */
    private boolean hasCalibratedOnceWhenNear = false;

    /** Whether to trust camera measurements for scoring mode decisions. */
    public boolean trustCameras = true;

    /** The current coral scoring position setting (at branch, one from branch, L1, L2). */
    public RobotPoses.Reef.RobotScoringSetting currentRobotScoringSetting = RobotPoses.Reef.RobotScoringSetting.ONE_CORAL_FROM_BRANCH;

    /** Whether L1 scoring mode is manually overridden. */
    private boolean l1RobotScoringSettingOverride = false;

    /** Whether L2 scoring mode is manually overridden. */
    private boolean l2RobotScoringSettingOverride = false;

    /** Whether the nearest algae target is on the high reef position. */
    public boolean nearestAlgaeIsHigh = false;

    /** The best coral pose detected by the color camera. */
    public Pose2d bestCoralPose = new Pose2d();

    /** The nearest robot pose at the closest reef branch (for scoring). */
    public Pose2d nearestRobotPoseAtBranch = new Pose2d();

    /** The pair of nearest robot poses at the two closest reef branches (left/right). */
    public Pair<Pose2d, Pose2d> nearestRobotPosesAtBranchPair = new Pair<>(new Pose2d(), new Pose2d());

    /** The pair of near-branch approach poses for the two closest reef branches. */
    public Pair<Pose2d, Pose2d> nearestRobotPosesNearBranchPair = new Pair<>(new Pose2d(), new Pose2d());

    /** The nearest reef tag pose considering both reef halves. */
    public Pose2d nearestReefTagPoseBothReefs = new Pose2d();

    /** The randomized robot pose at the net for varied scoring approaches. */
    public Pose2d randomizedRobotPoseAtNet = new Pose2d();

    /** The center robot pose at the net for direct scoring approaches. */
    public Pose2d nearestRobotPoseAtNetCenter = new Pose2d();

    /** The robot pose at the processor for the current alliance side. */
    public Pose2d currentAllianceSideRobotPoseAtProcessor = new Pose2d();

    /** The nearest robot pose at a coral station for intake. */
    public Pose2d nearestRobotPoseAtCoralStation = new Pose2d();

    /** The nearest robot pose at the algae reef for algae removal. */
    public Pose2d nearestRobotPoseAtAlgaeReef = new Pose2d();

    /** The near-approach pose to the algae reef. */
    public Pose2d nearestRobotPoseNearAlgaeReef = new Pose2d();

    /**
     * Constructs the Localizer.
     * <p>
     * Initializes the pose estimator, configures the localization strategy chooser
     * on SmartDashboard, and sets up the QuestNav and Limelight offsets.
     *
     * @param swerve The swerve drivetrain subsystem.
     */
    public Localizer(Swerve swerve) {
        this.swerve = swerve;

        localizationChooser.setDefaultOption("Pose Estimator", LocalizationStrategy.POSE_ESTIMATOR);
        localizationChooser.addOption("Quest Nav", LocalizationStrategy.QUEST_NAV);
        SmartDashboard.putData("Localization Strategy Chooser", localizationChooser);

        poseEstimator = new SwerveDrivePoseEstimator(
                this.swerve.getKinematics(),
                this.swerve.getState().RawHeading,
                this.swerve.getState().ModulePositions,
                this.swerve.getState().Pose,
                Constants.VisionConstants.ODOM_STD_DEV,
                Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(1.0)
        );

        configureQuestOffset();
        LimelightUtil.configureRobotToCameraOffset();
    }

    /**
     * Returns the robot pose from the currently active localization strategy.
     *
     * @return The current strategy pose (QuestNav or pose estimator).
     */
    public Pose2d getStrategyPose() {
        return strategy == LocalizationStrategy.QUEST_NAV ? getQuestPose() : getEstimatedPose();
    }

    /**
     * Returns the name of the current localization strategy.
     *
     * @return The strategy name ("POSE_ESTIMATOR" or "QUEST_NAV").
     */
    public String getLocalizationStrategy() {
        return strategy.name();
    }

    /**
     * Returns the current temporary target pose.
     *
     * @return The temporary target pose for pathfinding.
     */
    public Pose2d getCurrentTemporaryTargetPose() {
        return currentTemporaryTargetPose;
    }

    /**
     * Checks if QuestNav has calibrated once when the robot was near a tag.
     *
     * @return True if QuestNav has been calibrated at least once when nearby.
     */
    public boolean hasCalibratedOnceWhenNear() {
        return hasCalibratedOnceWhenNear;
    }

    /**
     * Returns the estimated robot pose from the pose estimator.
     *
     * @return The estimated pose.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the robot pose from the QuestNav headset.
     *
     * @return The QuestNav pose.
     */
    public Pose2d getQuestPose() {
        return QuestNavUtil.getRobotPose();
    }

    /**
     * Returns the heading toward the nearest coral station.
     *
     * @return The coral station heading in degrees.
     */
    public double getNearestCoralStationHeading() {
        return nearestRobotPoseAtCoralStation.getRotation().getDegrees();
    }

    /**
     * Returns the heading toward the nearest reef side (rotated 180 degrees to face the reef).
     *
     * @return The reef-facing heading in degrees.
     */
    public double getNearestReefSideHeading() {
        return nearestReefTagPoseBothReefs.getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    /**
     * Returns the heading for scoring at the processor.
     *
     * @return The processor scoring heading in degrees.
     */
    public double getProcessorScoringHeading() {
        return currentAllianceSideRobotPoseAtProcessor.getRotation().getDegrees();
    }

    /**
     * Returns the heading for scoring at the net.
     *
     * @return The net scoring heading in degrees.
     */
    public double getNetScoringHeading() {
        return nearestRobotPoseAtNetCenter.getRotation().getDegrees();
    }

    /**
     * Randomizes the net scoring pose by interpolating between inner and outer positions.
     *
     * @return The randomized net scoring pose.
     */
    public Pose2d randomizeNetScoringPose() {
        Pose2d currentPose = getStrategyPose();
        randomizedRobotPoseAtNet = RobotPoses.AlgaeScoring.getInnermostRobotPoseAtNet(currentPose).interpolate(
                RobotPoses.AlgaeScoring.getOutermostRobotPoseAtNet(currentPose),
                Math.random()
        );
        return randomizedRobotPoseAtNet;
    }

    /**
     * Sets the net scoring pose to the center position.
     *
     * @return The center net scoring pose.
     */
    public Pose2d centerNetScoringPose() {
        randomizedRobotPoseAtNet = nearestRobotPoseAtNetCenter;
        return randomizedRobotPoseAtNet;
    }

    /**
     * Returns the straight-line distance from the current pose to the action location for the given state.
     *
     * @param robotState The target robot state.
     * @return The distance in meters.
     */
    public double getDistanceToActionLocation(RobotStates.State robotState) {
        Pose2d currentPose = getStrategyPose();
        return switch (robotState) {
            case L1_CORAL, L2_CORAL, L3_CORAL, L4_CORAL -> currentPose.getTranslation().getDistance(nearestRobotPoseAtBranch.getTranslation());
            case PROCESSOR -> currentPose.getTranslation().getDistance(currentAllianceSideRobotPoseAtProcessor.getTranslation());
            case NET -> currentPose.getTranslation().getDistance(randomizedRobotPoseAtNet.getTranslation());
            case CORAL_STATION -> currentPose.getTranslation().getDistance(nearestRobotPoseAtCoralStation.getTranslation());
            case LOW_REEF_ALGAE, HIGH_REEF_ALGAE -> currentPose.getTranslation().getDistance(nearestRobotPoseAtAlgaeReef.getTranslation());
            default -> 0.0;
        };
    }

    /**
     * Returns the robot-relative translation vector from the current pose to the action location.
     *
     * @param robotState The target robot state.
     * @return The robot-relative translation to the action location.
     */
    public Translation2d getRobotRelativeVectorToActionLocation(RobotStates.State robotState) {
        Pose2d currentPose = getStrategyPose();
        return switch (robotState) {
            case L1_CORAL, L2_CORAL, L3_CORAL, L4_CORAL ->
                    nearestRobotPoseAtBranch.minus(new Pose2d(currentPose.getTranslation(), nearestRobotPoseAtBranch.getRotation())).getTranslation();
            case PROCESSOR ->
                    currentAllianceSideRobotPoseAtProcessor.minus(new Pose2d(currentPose.getTranslation(), currentAllianceSideRobotPoseAtProcessor.getRotation())).getTranslation();
            case NET ->
                    randomizedRobotPoseAtNet.minus(new Pose2d(currentPose.getTranslation(), randomizedRobotPoseAtNet.getRotation())).getTranslation();
            case CORAL_STATION ->
                    nearestRobotPoseAtCoralStation.minus(new Pose2d(currentPose.getTranslation(), nearestRobotPoseAtCoralStation.getRotation())).getTranslation();
            default -> new Translation2d();
        };
    }

    /**
     * Checks if the robot is facing away from the reef (orientation difference > 90 degrees).
     *
     * @return True if facing away from the reef.
     */
    public boolean facingAwayFromReef() {
        return Math.abs(nearestReefTagPoseBothReefs.getRotation().minus(getStrategyPose().getRotation()).getDegrees()) < 90.0;
    }

    /**
     * Checks if the robot is pressed against the reef wall (within 1 inch laterally).
     *
     * @return True if against the reef wall or cameras are not trusted.
     */
    public boolean isAgainstReefWall() {
        return !trustCameras || Math.abs(getRobotRelativeVectorToActionLocation(RobotStates.State.L4_CORAL).getX()) < Units.inchesToMeters(1.0);
    }

    /**
     * Checks if the robot is pressed against the coral station (within 0.22m laterally).
     *
     * @return True if against the coral station or cameras are not trusted.
     */
    public boolean isAgainstCoralStation() {
        return !trustCameras || Math.abs(getRobotRelativeVectorToActionLocation(RobotStates.State.CORAL_STATION).getX()) < 0.22;
    }

    /**
     * Checks if the robot is on the same side of the reef as the given scoring location.
     *
     * @param scoringLocation The reef scoring location to check against.
     * @return True if on the same side.
     */
    public boolean sameSideAsReefScoringLocation(FieldUtil.Reef.ScoringLocation scoringLocation) {
        return RobotPoses.Reef.sameSide(getStrategyPose(), RobotPoses.Reef.getRobotPoseAtBranch(currentRobotScoringSetting, scoringLocation));
    }

    /**
     * Checks if the robot is on the same side of the reef as the target pose.
     *
     * @param targetPose The target pose to compare sides with.
     * @return True if on the same side.
     */
    public boolean sameSideAsTarget(Pose2d targetPose) {
        return RobotPoses.Reef.sameSide(getStrategyPose(), targetPose);
    }

    /**
     * Checks if the robot has reached the transition state location (when to switch from pathfinding to direct drive).
     *
     * @param robotState The target robot state.
     * @param auto Whether in autonomous mode (uses tighter tolerance).
     * @return True if within transition tolerance.
     */
    public boolean atTransitionStateLocation(RobotStates.State robotState, boolean auto) {
        if (auto) {
            return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_TRANSITION_AUTO;
        }
        return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_TRANSITION;
    }

    /**
     * Checks if the robot is near enough to the target to begin direct drive.
     *
     * @param robotState The target robot state.
     * @return True if within direct drive tolerance.
     */
    public boolean nearStateLocation(RobotStates.State robotState) {
        return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE;
    }

    /**
     * Checks if the robot has reached the scoring location.
     *
     * @param robotState The target robot state.
     * @return True if within scoring tolerance.
     */
    public boolean atScoringLocation(RobotStates.State robotState) {
        return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
    }

    /** Toggles whether camera measurements are trusted for scoring decisions. */
    public void toggleTrustCameras() {
        trustCameras = !trustCameras;
    }

    /**
     * Sets whether L1 scoring mode is overridden.
     *
     * @param override True to force L1 scoring mode.
     */
    public void setL1RobotScoringSettingOverride(boolean override) {
        l1RobotScoringSettingOverride = override;
    }

    /**
     * Sets whether L2 scoring mode is overridden.
     *
     * @param override True to force L2 scoring mode.
     */
    public void setL2RobotScoringSettingOverride(boolean override) {
        l2RobotScoringSettingOverride = override;
    }

    /**
     * Sets the current temporary target pose.
     *
     * @param temporaryTargetPose The new temporary target pose.
     */
    public void setCurrentTemporaryTargetPose(Pose2d temporaryTargetPose) {
        this.currentTemporaryTargetPose = temporaryTargetPose;
    }

    /** Reads the localization strategy from the SmartDashboard chooser and updates if changed. */
    public void setLocalizationStrategyFromChooser() {
        LocalizationStrategy strategy = localizationChooser.getSelected();
        if (this.strategy != strategy) {
            this.strategy = strategy;
        }
    }

    /** Toggles between QuestNav and pose estimator localization strategies. */
    public void toggleLocalizationStrategy() {
        strategy = strategy == LocalizationStrategy.QUEST_NAV ? LocalizationStrategy.POSE_ESTIMATOR : LocalizationStrategy.QUEST_NAV;
    }

    /** Calibrates the QuestNav offset to the current pose estimator position. */
    public void configureQuestOffset() {
        QuestNavUtil.setQuestPose(poseEstimator.getEstimatedPosition());
    }

    /**
     * Resets all pose sources to the given pose.
     *
     * @param pose The pose to reset to.
     */
    public void setPoses(Pose2d pose) {
        poseEstimator.resetPose(pose);
        swerve.resetPose(pose);
        QuestNavUtil.setQuestPose(pose);
    }

    /**
     * Resets all heading sources to the given rotation.
     *
     * @param heading The rotation to reset to.
     */
    public void setRotations(Rotation2d heading) {
        swerve.resetRotation(heading);
        poseEstimator.resetRotation(heading);
        QuestNavUtil.setQuestPose(new Pose2d(getQuestPose().getTranslation(), heading));
    }

    /** Synchronizes all heading sources with the current pose estimator rotation. */
    public void syncRotations() {
        setRotations(poseEstimator.getEstimatedPosition().getRotation());
    }

    /** Adds Limelight MegaTag vision measurements to the pose estimator when tags are visible. */
    public void updateLimelightPoseEstimation() {
        if (LimelightUtil.isMultiTag() && LimelightUtil.isTagClear()) {
            Pose2d megaTagPose = LimelightUtil.getMegaTagOnePose();
            poseEstimator.addVisionMeasurement(
                    megaTagPose,
                    Timer.getFPGATimestamp() - LimelightUtil.getLatency(),
                    Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(LimelightUtil.getNearestTagDist())
            );
        }
    }

    /** Adds PhotonVision tag-based pose measurements from all BW cameras to the pose estimator. */
    public void updatePhotonPoseEstimation() {
        PhotonUtil.updateResults(poseEstimator.getEstimatedPosition().getRotation());
        for (PhotonUtil.BW.BWCamera camera : PhotonUtil.BW.BWCamera.values()) {
            if (PhotonUtil.BW.isTagClear(camera)) {
                Optional<EstimatedRobotPose> optionalPoseEstimate = PhotonUtil.BW.getBestTagPose(camera);
                optionalPoseEstimate.ifPresent(
                        poseEstimate -> poseEstimator.addVisionMeasurement(
                                poseEstimate.estimatedPose().toPose2d(),
                                poseEstimate.timestampSeconds(),
                                poseEstimate.stdDevs()
                        )
                );
            }
        }
    }

    /**
     * Updates the QuestNav pose offset based on error between pose estimate and QuestNav pose.
     * <p>
     * Calibrates when the robot is stationary, near tags, and has clear vision.
     */
    public void updateQuestNavPose() {
        QuestNavUtil.completeQuestPose();
        if (LimelightUtil.getNearestTagDist() > Constants.VisionConstants.QuestNavConstants.MIN_TAG_DIST_TO_BE_FAR) {
            hasCalibratedOnceWhenNear = false;
        }
        if (!hasCalibratedOnceWhenNear) {
            if (LimelightUtil.isTagClear() && PhotonUtil.BW.isTagClear()
                    && this.swerve.getState().Speeds.vxMetersPerSecond == 0
                    && this.swerve.getState().Speeds.vyMetersPerSecond == 0
                    && Math.abs(this.swerve.getState().Speeds.omegaRadiansPerSecond) == 0) {
                configureQuestOffset();
                hasCalibratedOnceWhenNear = true;
            }
        }
    }

    /** Forces an immediate QuestNav pose calibration regardless of conditions. */
    public void forceUpdateQuestNavPose() {
        hasCalibratedOnceWhenNear = false;
        updateQuestNavPose();
    }

    /** Updates the coral scoring mode based on camera trust and override settings. */
    private void updateCoralScoringMode() {
        if (!trustCameras) {
            currentRobotScoringSetting = RobotPoses.Reef.RobotScoringSetting.AT_BRANCH;
        } else if (l1RobotScoringSettingOverride) {
           currentRobotScoringSetting = RobotPoses.Reef.RobotScoringSetting.L1;
        } else if (l2RobotScoringSettingOverride) {
            currentRobotScoringSetting = RobotPoses.Reef.RobotScoringSetting.L2;
        } else {
            currentRobotScoringSetting = RobotPoses.Reef.RobotScoringSetting.ONE_CORAL_FROM_BRANCH;
        }
    }

    /** Recalculates all utility poses (reef branches, coral stations, algae, net, processor) based on current pose. */
    private void updateRobotUtilityPoses() {
        Pose2d currentPose = getStrategyPose();

        updateCoralScoringMode();
        nearestAlgaeIsHigh = FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(currentPose, true)) == FieldUtil.Reef.AlgaeLocation.HIGH;

        PhotonUtil.Color.getRobotToBestObject(PhotonUtil.Color.TargetClass.CORAL).ifPresent(robotToObject ->
                bestCoralPose = new Pose2d(
                        currentPose.plus(new Transform2d(robotToObject, Rotation2d.kZero)).getTranslation(),
                        currentPose.getRotation().rotateBy(robotToObject.getAngle()).rotateBy(Rotation2d.kPi)
                ).plus(new Transform2d(
                        Constants.ROBOT_LENGTH_WITH_BUMPERS.in(Meters) / 2 + Units.inchesToMeters(12.0), // TODO SHOP: TUNE THIS
                        0,
                        Rotation2d.kZero
                ))
        );

        nearestRobotPoseAtBranch = RobotPoses.Reef.getNearestRobotPoseAtBranch(currentRobotScoringSetting, currentPose);
        nearestRobotPosesAtBranchPair = RobotPoses.Reef.getNearestRobotPosesAtBranchPair(currentRobotScoringSetting, currentPose);
        nearestRobotPosesNearBranchPair = RobotPoses.Reef.getNearestRobotPosesNearBranchPair(currentRobotScoringSetting, currentPose);
        nearestReefTagPoseBothReefs = FieldUtil.Reef.getNearestReefTagPose(currentPose, true);

        nearestRobotPoseAtNetCenter = RobotPoses.AlgaeScoring.getRobotPoseAtNetCenter(currentPose);
        currentAllianceSideRobotPoseAtProcessor = RobotPoses.AlgaeScoring.getCurrentAllianceSideRobotPoseAtProcessor(currentPose);
        nearestRobotPoseAtCoralStation = getStrategyPose().nearest(List.of(
                RobotPoses.CoralStation.getRobotPosesAtEachCoralStation().get(0).interpolate(Constants.FAR_LEFT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER), 0.25),
                RobotPoses.CoralStation.getRobotPosesAtEachCoralStation().get(1).interpolate(Constants.FAR_RIGHT_CORAL_STATION.apply(Constants.ALLIANCE_SUPPLIER), 0.25)
        ));
        nearestRobotPoseAtAlgaeReef = RobotPoses.Reef.getNearestRobotPoseAtAlgaeReef(currentPose, nearestAlgaeIsHigh);
        nearestRobotPoseNearAlgaeReef = RobotPoses.Reef.getNearestRobotPoseNearReef(nearestAlgaeIsHigh, currentPose);
    }

    /**
     * Periodically updates the localization estimator and auxiliary pose data.
     *
     * <p>This method publishes localization telemetry, updates the pose estimator
     * with odometry and vision measurements, processes PhotonVision and Limelight
     * updates, selects the active localization strategy from the SmartDashboard
     * chooser, and recalculates utility poses used by autonomous routines
     * (nearest reef branches, coral stations, algae poses, net/processor poses).
     */
    public void periodic() {
        localizationTelemetry.publishValues();

        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        updatePhotonPoseEstimation();

        setLocalizationStrategyFromChooser();

        updateRobotUtilityPoses();
    }
}