package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.telemetry.VisionTelemetry;
import frc.robot.util.TagLocation;
import frc.robot.util.VisionUtil;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class Localizer {
    private enum PoseMode {
        POSE_ESTIMATOR,
        QUEST_NAV
    }

    // localizer is a dependent of swerve
    private final Swerve swerve;
    private final VisionTelemetry visionTelemetry;

    private final SwerveDrivePoseEstimator poseEstimator;
    // Photon Vision's integrated estimator, to be integrated into the above estimator
    private final PhotonPoseEstimator photonPoseEstimator;

    // Transformation applied to QuestNav pose to adjust origin to the pose estimator's origin
    private Translation2d questTransOffset = new Translation2d();
    private Rotation2d questRotOffset = new Rotation2d();

    // The pose extrapolation method that the robot will use. It will be set to QuestNav by default.
    private PoseMode localizationMode = PoseMode.QUEST_NAV;

    private boolean isMegaTagTwoConfigured = false;

    public Localizer(Swerve swerve) {
        this.swerve = swerve;

        visionTelemetry = new VisionTelemetry(this);

        poseEstimator = new SwerveDrivePoseEstimator(
                this.swerve.getKinematics(),
                this.swerve.getState().RawHeading,
                this.swerve.getState().ModulePositions,
                this.swerve.getState().Pose,
                Constants.VisionConstants.ODOM_STD_DEV,
                Constants.VisionConstants.VISION_STD_DEV_UNCONFIGURED
        );

        photonPoseEstimator = new PhotonPoseEstimator(
                VisionUtil.Photon.BW.tagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionUtil.Photon.BW.robotToCameraOffset
        );

        configureQuestOffset();
        VisionUtil.Limelight.configureRobotToCameraOffset();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public boolean highConfidenceEstimate() {
        return VisionUtil.Limelight.isTagClear(); // TODO IMPLEMENT THIS ONCE BW IS SET UP && VisionUtil.Photon.BW.isTagClear();
    }

    public boolean isMegaTagTwoConfigured() {
        return isMegaTagTwoConfigured;
    }

    public Pose2d getQuestCorrectedPose() {
        return new Pose2d(
                getQuestCorrectedTranslation(),
                getQuestCorrectedRotation()
        );
    }

    public Translation2d getQuestCorrectedTranslation() {
        Pose2d rawPose = VisionUtil.QuestNav.getRawPose();
        return questTransOffset.plus(new Translation2d(
                rawPose.getX() * Math.cos(questRotOffset.getRadians()) - rawPose.getY() * Math.sin(questRotOffset.getRadians()),
                rawPose.getX() * Math.sin(questRotOffset.getRadians()) + rawPose.getY() * Math.cos(questRotOffset.getRadians())
        ));
    }

    public Rotation2d getQuestCorrectedRotation() {
        return questRotOffset.rotateBy(VisionUtil.QuestNav.getRawPose().getRotation());
    }

    public Translation2d getQuestTransOffset() {
        return questTransOffset;
    }

    public Rotation2d getQuestRotOffset() {
        return questRotOffset;
    }

    public Pose2d getModePose() {
        return localizationMode == PoseMode.QUEST_NAV ? getQuestCorrectedPose() : getEstimatedPose();
    }

    public boolean isQuestMode() {
        return localizationMode == PoseMode.QUEST_NAV;
    }

    public Translation2d getTranslationToSpeaker() {
        Translation2d robotTranslation = getModePose().getTranslation();
        Translation2d tagTranslation = TagLocation.getSpeakerTagPose().getTranslation();
        return tagTranslation.minus(robotTranslation);
    }

    public double getAngleToSpeaker() {
        return getTranslationToSpeaker().getAngle().getDegrees();
    }

    public void switchMode() {
        localizationMode = localizationMode == PoseMode.QUEST_NAV ? PoseMode.POSE_ESTIMATOR : PoseMode.QUEST_NAV;
    }

    public void recalibrate() {
        isMegaTagTwoConfigured = false;
        poseEstimator.resetPose(new Pose2d());
    }

    public void configureQuestOffset() {
        if (highConfidenceEstimate()) {
            questRotOffset = getEstimatedPose().getRotation().minus(VisionUtil.QuestNav.getRawPose().getRotation());
            questTransOffset = getEstimatedPose().getTranslation();
        }
    }

    public void setQuestNavPose(Pose2d pose) {
        questRotOffset = pose.getRotation().minus(VisionUtil.QuestNav.getRawPose().getRotation());
        questTransOffset = pose.getTranslation();
    }

    public void setPoses(Pose2d pose) {
        this.swerve.resetPose(pose);
        poseEstimator.resetPose(pose);
        setQuestNavPose(pose);
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
                poseEstimator.setVisionMeasurementStdDevs(Constants.VisionConstants.VISION_STD_DEV_CONFIGURED);
                isMegaTagTwoConfigured = true;
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
        if (!VisionUtil.Photon.BW.hasTargets()) { return; }
        Optional<EstimatedRobotPose> photonPose = photonPoseEstimator.update(VisionUtil.Photon.BW.latestResult);
        // TODO TEST IF OTHER DOESN'T WORK Pose2d photonPose = VisionUtil.Photon.BW.getPhotonPose();
        if (VisionUtil.Photon.BW.isTagClear() && photonPose.isPresent()) {
            poseEstimator.addVisionMeasurement(
                    photonPose.get().estimatedPose.toPose2d(),
                    photonPose.get().timestampSeconds
                    // TODO TEST IF OTHER DOESN'T WORK VisionUtil.Photon.BW.getTimestamp()
            );
        }
    }

    public void updatePoseEstimation() {
        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        updateLimelightPoseEstimation();
        updatePhotonPoseEstimation();
    }

    // changes offset based on error between pose estimate and corrected QuestNav pose
    public void updateQuestNavPose() {
        if (highConfidenceEstimate()) {
            // Accumulated error between pose estimator and corrected QuestNav pose
            Transform2d correctionError = getEstimatedPose().minus(getQuestCorrectedPose());
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
        visionTelemetry.publishValues();
    }
}
