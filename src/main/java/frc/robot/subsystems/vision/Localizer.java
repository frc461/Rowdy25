package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.FieldUtil;
import frc.robot.util.VisionUtil;

import java.util.Optional;

public class Localizer {
    private enum LocalizationStrategy {
        POSE_ESTIMATOR,
        QUEST_NAV
    }

    // localizer is a dependent of swerve
    private final Swerve swerve;
    private final LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry(this);
    private final SendableChooser<LocalizationStrategy> localizationChooser = new SendableChooser<>();

    private final SwerveDrivePoseEstimator poseEstimator;

    // The pose extrapolation method that the robot will use. It will be set to QuestNav by default.
    private LocalizationStrategy strategy = LocalizationStrategy.POSE_ESTIMATOR;

    private boolean hasCalibratedOnceWhenNear = false;

    public Localizer(Swerve swerve) {
        this.swerve = swerve;

        localizationTelemetry.registerListeners();
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
        VisionUtil.Limelight.configureRobotToCameraOffset();
    }

    public Pose2d getStrategyPose() {
        return strategy == LocalizationStrategy.QUEST_NAV ? getQuestPose() : getEstimatedPose();
    }

    public String getLocalizationStrategy() {
        return strategy == LocalizationStrategy.QUEST_NAV ? "Quest Nav" : "Pose Estimator";
    }

    public boolean hasCalibratedOnceWhenNear() {
        return hasCalibratedOnceWhenNear;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getQuestPose() {
        return VisionUtil.QuestNav.getFinalRobotPose();
    }

    public Translation2d getTranslationToSpeaker() {
        Translation2d robotTranslation = getStrategyPose().getTranslation();
        Translation2d tagTranslation = FieldUtil.TagLocation.getSpeakerTagPose().getTranslation();
        return tagTranslation.minus(robotTranslation);
    }

    public double getAngleToSpeaker() {
        return getTranslationToSpeaker().getAngle().getDegrees();
    }

    public void setLocalizationStrategyFromChooser() {
        strategy = localizationChooser.getSelected();
    }

    public void toggleLocalizationStrategy() {
        strategy = strategy == LocalizationStrategy.QUEST_NAV ? LocalizationStrategy.POSE_ESTIMATOR : LocalizationStrategy.QUEST_NAV;
    }

    public void configureQuestOffset() {
        VisionUtil.QuestNav.setQuestPose(poseEstimator.getEstimatedPosition());
    }

    public void setPoses(Pose2d pose) {
        this.swerve.resetPose(pose);
        poseEstimator.resetPose(pose);
        VisionUtil.QuestNav.setQuestPose(pose);
    }

    public void updateLimelightPoseEstimation() {
        if (VisionUtil.Limelight.isMultiTag() && VisionUtil.Limelight.isTagClear()) {
            Pose2d megaTagPose = VisionUtil.Limelight.getMegaTagOnePose();
            poseEstimator.addVisionMeasurement(
                    megaTagPose,
                    Timer.getFPGATimestamp() - VisionUtil.Limelight.getLatency(),
                    Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(VisionUtil.Limelight.getNearestTagDist())
            );
        }
    }

    public void updatePhotonPoseEstimation() {
        VisionUtil.Photon.updateResults();
        if (VisionUtil.Photon.BW.isTagClear()) {
            if (VisionUtil.Photon.BW.isMultiTag()) {
                VisionUtil.Photon.BW.getOptionalPoseData().ifPresent(photonPose -> poseEstimator.addVisionMeasurement(
                        photonPose.estimatedPose.toPose2d(),
                        photonPose.timestampSeconds,
                        Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(VisionUtil.Photon.BW.getBestTagDist())
                ));
                return;
            }

            Optional<Pose2d> photonPose = VisionUtil.Photon.BW.getSingleTagPose(poseEstimator.getEstimatedPosition());
            if (photonPose.isEmpty()) { return; }
            poseEstimator.addVisionMeasurement(
                    photonPose.get(),
                    VisionUtil.Photon.BW.getLatestResultTimestamp(),
                    Constants.VisionConstants.VISION_STD_DEV_FUNCTION.apply(VisionUtil.Photon.BW.getBestTagDist())
            );
        }
    }

    public void updatePoseEstimation() {
        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        updateLimelightPoseEstimation();
        updatePhotonPoseEstimation();
    }

    public void forceUpdateQuestNavPose() {
        hasCalibratedOnceWhenNear = false;
        updateQuestNavPose();
    }

    // changes offset based on error between pose estimate and corrected QuestNav pose
    public void updateQuestNavPose() {
        VisionUtil.QuestNav.completeQuestPose();
        if (VisionUtil.Limelight.getNearestTagDist() > Constants.VisionConstants.QuestNavConstants.MIN_TAG_DIST_TO_BE_FAR) {
            hasCalibratedOnceWhenNear = false;
        }
        if (!hasCalibratedOnceWhenNear) {
            if (VisionUtil.highConfidenceEstimation()
                    && this.swerve.getState().Speeds.vxMetersPerSecond == 0
                    && this.swerve.getState().Speeds.vyMetersPerSecond == 0
                    && Math.abs(this.swerve.getState().Speeds.omegaRadiansPerSecond) == 0) {
                configureQuestOffset();
                hasCalibratedOnceWhenNear = true;
            }
        }
    }

     public void updatePoses() {
         updatePoseEstimation();
         updateQuestNavPose();
     }

    public void periodic() {
        updatePoses();
        localizationTelemetry.publishValues();
    }
}
