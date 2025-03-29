package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotStates;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.*;
import frc.robot.util.vision.LimelightUtil;
import frc.robot.util.vision.PhotonUtil;
import frc.robot.util.vision.QuestNavUtil;

import java.util.Optional;

public class Localizer {
    private enum LocalizationStrategy {
        POSE_ESTIMATOR,
        QUEST_NAV
    }

    // localizer is a dependent of swerve
    private final Swerve swerve;
    private final DigitalInput proximitySensor = new DigitalInput(Constants.VisionConstants.PROXIMITY_SENSOR_DIO_PORT); // TODO SHOP: TEST AND CONFIGURE CANANDCOLOR
    private final LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry(this);
    private final SendableChooser<LocalizationStrategy> localizationChooser = new SendableChooser<>();

    private final SwerveDrivePoseEstimator poseEstimator;

    // The pose extrapolation method that the robot will use. It will be set to the pose estimator by default.
    private LocalizationStrategy strategy = LocalizationStrategy.POSE_ESTIMATOR;

    private Pose2d currentTemporaryTargetPose = new Pose2d();
    private boolean hasCalibratedOnceWhenNear = false;

    public final Pose2d robotPoseAtProcessor;
    public final Pose2d robotPoseAtNet;
    public Pose2d nearestRobotPoseAtCoralStation = new Pose2d();
    public Pose2d nearestRobotPoseAtAlgaeReef = new Pose2d();
    public Pose2d nearestRobotPoseAtBranch = new Pose2d();
    public Pose2d nearestRobotPoseAtBranchUsingReefCenter = new Pose2d();
    public Pair<Pose2d, Pose2d> nearestRobotPosesAtBranchPair = new Pair<>(new Pose2d(), new Pose2d());
    public Pair<Pose2d, Pose2d> nearestRobotPosesAtBranchPairUsingReefCenter = new Pair<>(new Pose2d(), new Pose2d());
    public Pose2d nearestReefTagPose = new Pose2d();
    public boolean nearestAlgaeIsHigh = false;

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

        robotPoseAtProcessor = FieldUtil.AlgaeScoring.getRobotPoseAtProcessor();
        robotPoseAtNet = FieldUtil.AlgaeScoring.getRobotPoseAtNet();
    }

    public boolean isNearWall() {
        return proximitySensor.get();
    }

    public Pose2d getStrategyPose() {
        return strategy == LocalizationStrategy.QUEST_NAV ? getQuestPose() : getEstimatedPose();
    }

    public String getLocalizationStrategy() {
        return strategy == LocalizationStrategy.QUEST_NAV ? "Quest Nav" : "Pose Estimator";
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
        return nearestReefTagPose.getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    public double getDistanceToScoringLocation(RobotStates.State robotState) {
        Pose2d currentPose = getStrategyPose();
        return switch (robotState) {
            case L1_CORAL, L2_CORAL, L3_CORAL, L4_CORAL -> currentPose.getTranslation().getDistance(nearestRobotPoseAtBranch.getTranslation());
            case PROCESSOR -> currentPose.getTranslation().getDistance(robotPoseAtProcessor.getTranslation());
            case NET -> currentPose.getTranslation().getDistance(robotPoseAtNet.getTranslation());
            case CORAL_STATION -> currentPose.getTranslation().getDistance(nearestRobotPoseAtCoralStation.getTranslation());
            default -> 0.0;
        };
    }

    public boolean atScoringLocation(RobotStates.State robotState) {
        return getDistanceToScoringLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
    }

    public boolean nearStateLocation(RobotStates.State robotState) {
        return getDistanceToScoringLocation(robotState) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_TRANSITION;
    }

    public double getProcessorScoringHeading() {
        return robotPoseAtProcessor.getRotation().getDegrees();
    }

    public double getNetScoringHeading() {
        return robotPoseAtNet.getRotation().getDegrees();
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
                Optional<EstimatedRobotPose> optionalPoseEstimate = getUpdatedPhotonPoseEstimate(camera);
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

    public Optional<EstimatedRobotPose> getUpdatedPhotonPoseEstimate(PhotonUtil.BW.BWCamera camera) {
        return PhotonUtil.BW.getBestTagPose(camera);
    }

    public void forceUpdateQuestNavPose() {
        hasCalibratedOnceWhenNear = false;
        updateQuestNavPose();
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

    private void updateFieldUtilityPoses() {
        nearestRobotPoseAtCoralStation = FieldUtil.CoralStation.getNearestRobotPoseAtCoralStation(getStrategyPose());
        nearestRobotPoseAtAlgaeReef = FieldUtil.Reef.getNearestRobotPoseAtAlgaeReef(getStrategyPose());
        nearestRobotPoseAtBranch = FieldUtil.Reef.getNearestRobotPoseAtBranch(getStrategyPose());
        nearestRobotPoseAtBranchUsingReefCenter = FieldUtil.Reef.getNearestRobotPoseAtBranchUsingReefCenter(getStrategyPose());
        nearestRobotPosesAtBranchPair = FieldUtil.Reef.getNearestRobotPosesAtBranchPair(getStrategyPose());
        nearestRobotPosesAtBranchPairUsingReefCenter = FieldUtil.Reef.getNearestRobotPosesAtBranchPairUsingReefCenter(getStrategyPose());
        nearestReefTagPose = FieldUtil.Reef.getNearestReefTagPose(getStrategyPose());
        nearestAlgaeIsHigh = FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(getStrategyPose())) == FieldUtil.Reef.AlgaeLocation.HIGH;
    }

    public void periodic() {
        localizationTelemetry.publishValues();

        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        updatePhotonPoseEstimation();
        updateQuestNavPose();

        setLocalizationStrategyFromChooser();

        updateFieldUtilityPoses();
    }
}
