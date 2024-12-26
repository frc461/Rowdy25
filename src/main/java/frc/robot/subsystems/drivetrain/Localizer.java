package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.telemetry.LocalizationTelemetry;
import frc.robot.util.TagLocation;
import frc.robot.util.VisionUtil;

public class Localizer {
    private enum LocalizationStrategy {
        POSE_ESTIMATOR,
        QUEST_NAV
    }

    // localizer is a dependent of swerve
    private final Swerve swerve;
    private final LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry(this);

    private final SwerveDrivePoseEstimator poseEstimator;

    // Transformation applied to QuestNav pose to adjust origin to the pose estimator's origin

    // The pose extrapolation method that the robot will use. It will be set to QuestNav by default.
    private LocalizationStrategy strategy = LocalizationStrategy.QUEST_NAV;

    private boolean isMegaTagTwoConfigured = false;

    public Localizer(Swerve swerve) {
        this.swerve = swerve;

        localizationTelemetry.registerListeners();

        poseEstimator = new SwerveDrivePoseEstimator(
                this.swerve.getKinematics(),
                this.swerve.getState().RawHeading,
                this.swerve.getState().ModulePositions,
                this.swerve.getState().Pose,
                Constants.VisionConstants.ODOM_STD_DEV,
                Constants.VisionConstants.VISION_STD_DEV_UNCONFIGURED
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

    public boolean isMegaTagTwoConfigured() {
        return isMegaTagTwoConfigured;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getQuestPose() {
        return VisionUtil.QuestNav.getFinalRobotPose();
    }

    public Translation2d getTranslationToSpeaker() {
        Translation2d robotTranslation = getStrategyPose().getTranslation();
        Translation2d tagTranslation = TagLocation.getSpeakerTagPose().getTranslation();
        return tagTranslation.minus(robotTranslation);
    }

    public double getAngleToSpeaker() {
        return getTranslationToSpeaker().getAngle().getDegrees();
    }

    public void toggleStrategy() {
        strategy = strategy == LocalizationStrategy.QUEST_NAV ? LocalizationStrategy.POSE_ESTIMATOR : LocalizationStrategy.QUEST_NAV;
    }

    public void recalibrateMegaTag() {
        isMegaTagTwoConfigured = false;
        poseEstimator.resetPose(new Pose2d());
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
        VisionUtil.Limelight.calibrateRobotOrientation(poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        if (!isMegaTagTwoConfigured && VisionUtil.Limelight.isTagClear()) {
            Pose2d megaTagOnePose = VisionUtil.Limelight.getMegaTagOnePose();
            Transform2d megaTagTwoOffset = VisionUtil.Limelight.getMegaTagTwoPose().minus(megaTagOnePose);
            poseEstimator.addVisionMeasurement(
                    megaTagOnePose,
                    Timer.getFPGATimestamp() - VisionUtil.Limelight.getLatency()
            );
            if (megaTagTwoOffset.getTranslation().getNorm() < Constants.VisionConstants.CONFIGURED_TRANSLATION_THRESHOLD) {
                isMegaTagTwoConfigured = true;
                poseEstimator.setVisionMeasurementStdDevs(Constants.VisionConstants.VISION_STD_DEV_CONFIGURED);
            }
            return;
        }

        if (VisionUtil.Limelight.isTagClear()
                && Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValue().in(edu.wpi.first.units.Units.DegreesPerSecond)) < Constants.VisionConstants.CONFIGURED_MAX_ANG_VEL
        ) {
            Pose2d megaTagTwoPose = VisionUtil.Limelight.getMegaTagTwoPose();
            poseEstimator.addVisionMeasurement(
                    megaTagTwoPose,
                    Timer.getFPGATimestamp() - VisionUtil.Limelight.getLatency()
            );
        }
    }

    public void updatePhotonPoseEstimation() {
        VisionUtil.Photon.updateResults();
        VisionUtil.Photon.BW.getOptionalPoseData().ifPresent(photonPose -> poseEstimator.addVisionMeasurement(
                photonPose.estimatedPose.toPose2d(),
                photonPose.timestampSeconds
        ));
    }

    public void updatePoseEstimation() {
        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        updateLimelightPoseEstimation();
        updatePhotonPoseEstimation();
    }

    // changes offset based on error between pose estimate and corrected QuestNav pose
    public void updateQuestNavPose() {
        VisionUtil.QuestNav.completeQuestPose();
        if (VisionUtil.highConfidenceEstimation()) {
            // Accumulated error between pose estimator and corrected QuestNav pose
            Transform2d correctionError = getEstimatedPose().minus(getQuestPose());
            double transDiff = correctionError.getTranslation().getNorm();
            double rotDiff = correctionError.getRotation().getDegrees();
            if (transDiff > Constants.VisionConstants.QuestNavConstants.TRANSLATION_ERROR_TOLERANCE
                    || rotDiff > Constants.VisionConstants.QuestNavConstants.ROTATION_ERROR_TOLERANCE) {
                configureQuestOffset();
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
