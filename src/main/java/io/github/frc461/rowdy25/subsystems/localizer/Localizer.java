package io.github.frc461.rowdy25.subsystems.localizer;

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
import frc.robot.util.*;
import io.github.frc461.rowdy25.util.EstimatedRobotPose;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.vision.LimelightUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;
import io.github.frc461.rowdy25.util.vision.QuestNavUtil;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

public class Localizer {
    private enum LocalizationStrategy {
        POSE_ESTIMATOR,
        QUEST_NAV
    }

    // localizer is a dependent of swerve
    private final Swerve swerve;
    private final DigitalInput proximitySensor = new DigitalInput(Constants.VisionConstants.PROXIMITY_SENSOR_DIO_PORT); // TODO SHOP: TEST MORE
    private final LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry(this);
    private final SendableChooser<LocalizationStrategy> localizationChooser = new SendableChooser<>();

    private final SwerveDrivePoseEstimator poseEstimator;

    // The pose extrapolation method that the robot will use. It will be set to the pose estimator by default.
    private LocalizationStrategy strategy = LocalizationStrategy.POSE_ESTIMATOR;

    private Pose2d currentTemporaryTargetPose = new Pose2d();
    private boolean hasCalibratedOnceWhenNear = false;

    public boolean trustCameras = true;

    public RobotPoses.Reef.RobotScoringSetting currentRobotScoringSetting = RobotPoses.Reef.RobotScoringSetting.ONE_CORAL_FROM_BRANCH;
    private boolean l1RobotScoringSettingOverride = false;
    private boolean l2RobotScoringSettingOverride = false;
    public boolean nearestAlgaeIsHigh = false;

    public Pose2d bestCoralPose = new Pose2d();

    public Pose2d nearestRobotPoseAtBranch = new Pose2d();
    public Pair<Pose2d, Pose2d> nearestRobotPosesAtBranchPair = new Pair<>(new Pose2d(), new Pose2d());
    public Pair<Pose2d, Pose2d> nearestRobotPosesNearBranchPair = new Pair<>(new Pose2d(), new Pose2d());
    public Pose2d nearestReefTagPoseBothReefs = new Pose2d();

    public Pose2d randomizedRobotPoseAtNet = new Pose2d();
    public Pose2d nearestRobotPoseAtNetCenter = new Pose2d();
    public Pose2d currentAllianceSideRobotPoseAtProcessor = new Pose2d();
    public Pose2d nearestRobotPoseAtCoralStation = new Pose2d();
    public Pose2d nearestRobotPoseAtAlgaeReef = new Pose2d();
    public Pose2d nearestRobotPoseNearAlgaeReef = new Pose2d();

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

    public Pose2d getStrategyPose() {
        return strategy == LocalizationStrategy.QUEST_NAV ? getQuestPose() : getEstimatedPose();
    }

    public String getLocalizationStrategy() {
        return strategy.name();
    }

    public Pose2d getCurrentTemporaryTargetPose() {
        return currentTemporaryTargetPose;
    }

    public boolean hasCalibratedOnceWhenNear() {
        return hasCalibratedOnceWhenNear;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getQuestPose() {
        return QuestNavUtil.getRobotPose();
    }

    public double getNearestCoralStationHeading() {
        return nearestRobotPoseAtCoralStation.getRotation().getDegrees();
    }

    public double getNearestReefSideHeading() {
        return nearestReefTagPoseBothReefs.getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    public double getProcessorScoringHeading() {
        return currentAllianceSideRobotPoseAtProcessor.getRotation().getDegrees();
    }

    public double getNetScoringHeading() {
        return nearestRobotPoseAtNetCenter.getRotation().getDegrees();
    }

    public Pose2d randomizeNetScoringPose() {
        Pose2d currentPose = getStrategyPose();
        randomizedRobotPoseAtNet = RobotPoses.AlgaeScoring.getInnermostRobotPoseAtNet(currentPose).interpolate(
                RobotPoses.AlgaeScoring.getOutermostRobotPoseAtNet(currentPose),
                Math.random()
        );
        return randomizedRobotPoseAtNet;
    }

    public Pose2d centerNetScoringPose() {
        randomizedRobotPoseAtNet = nearestRobotPoseAtNetCenter;
        return randomizedRobotPoseAtNet;
    }

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

    public boolean facingAwayFromReef() {
        return Math.abs(nearestReefTagPoseBothReefs.getRotation().minus(getStrategyPose().getRotation()).getDegrees()) < 90.0;
    }

    public boolean isAgainstReefWall() {
        return !trustCameras || Math.abs(getRobotRelativeVectorToActionLocation(RobotStates.State.L4_CORAL).getX()) < Units.inchesToMeters(1.0);
    }

    public boolean isAgainstCoralStation() {
        return !trustCameras || Math.abs(getRobotRelativeVectorToActionLocation(RobotStates.State.CORAL_STATION).getX()) < 0.22;
    }

    public boolean sameSideAsReefScoringLocation(FieldUtil.Reef.ScoringLocation scoringLocation) {
        return RobotPoses.Reef.sameSide(getStrategyPose(), RobotPoses.Reef.getRobotPoseAtBranch(currentRobotScoringSetting, scoringLocation));
    }

    public boolean sameSideAsTarget(Pose2d targetPose) {
        return RobotPoses.Reef.sameSide(getStrategyPose(), targetPose);
    }

    public boolean atTransitionStateLocation(RobotStates.State robotState, boolean auto) {
        if (auto) {
            return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_TRANSITION_AUTO;
        }
        return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_TRANSITION;
    }

    public boolean nearStateLocation(RobotStates.State robotState) {
        return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_DIRECT_DRIVE;
    }

    public boolean atScoringLocation(RobotStates.State robotState) {
        return getDistanceToActionLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
    }

    public void toggleTrustCameras() {
        trustCameras = !trustCameras;
    }

    public void setL1RobotScoringSettingOverride(boolean override) {
        l1RobotScoringSettingOverride = override;
    }

    public void setL2RobotScoringSettingOverride(boolean override) {
        l2RobotScoringSettingOverride = override;
    }

    public void setCurrentTemporaryTargetPose(Pose2d temporaryTargetPose) {
        this.currentTemporaryTargetPose = temporaryTargetPose;
    }

    public void setLocalizationStrategyFromChooser() {
        LocalizationStrategy strategy = localizationChooser.getSelected();
        if (this.strategy != strategy) {
            this.strategy = strategy;
        }
    }

    public void toggleLocalizationStrategy() {
        strategy = strategy == LocalizationStrategy.QUEST_NAV ? LocalizationStrategy.POSE_ESTIMATOR : LocalizationStrategy.QUEST_NAV;
    }

    public void configureQuestOffset() {
        QuestNavUtil.setQuestPose(poseEstimator.getEstimatedPosition());
    }

    public void setPoses(Pose2d pose) {
        poseEstimator.resetPose(pose);
        swerve.resetPose(pose);
        QuestNavUtil.setQuestPose(pose);
    }

    public void setRotations(Rotation2d heading) {
        swerve.resetRotation(heading);
        poseEstimator.resetRotation(heading);
        QuestNavUtil.setQuestPose(new Pose2d(getQuestPose().getTranslation(), heading));
    }

    public void syncRotations() {
        setRotations(poseEstimator.getEstimatedPosition().getRotation());
    }

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

    // changes offset based on error between pose estimate and corrected QuestNav pose
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

    public void forceUpdateQuestNavPose() {
        hasCalibratedOnceWhenNear = false;
        updateQuestNavPose();
    }

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

    public void periodic() {
        localizationTelemetry.publishValues();

        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        updatePhotonPoseEstimation();

        setLocalizationStrategyFromChooser();

        updateRobotUtilityPoses();
    }
}
