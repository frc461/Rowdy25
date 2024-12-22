package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
    private final Swerve swerve; // localizer is a dependent of swerve
    private final VisionTelemetry visionTelemetry;

    private final SwerveDrivePoseEstimator poseEstimator;
    // Photon Vision's integrated estimator, to be integrated into the above estimator
    private final PhotonPoseEstimator photonPoseEstimator;

    // Transformation applied to QuestNav pose to adjust origin to the pose estimator's origin
    private Transform2d questNavToPoseEstimateOffset = new Transform2d();

    public Localizer(Swerve swerve) {
        this.swerve = swerve;

        visionTelemetry = new VisionTelemetry(this);

        poseEstimator = new SwerveDrivePoseEstimator(
                this.swerve.getKinematics(),
                this.swerve.getState().RawHeading,
                this.swerve.getState().ModulePositions,
                this.swerve.getState().Pose,
                Constants.VisionConstants.ODOM_STD_DEV,
                Constants.VisionConstants.VISION_STD_DEV
        );

        photonPoseEstimator = new PhotonPoseEstimator(
                VisionUtil.Photon.BW.tagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionUtil.Photon.BW.robotToCameraOffset
        );

        setQuestNavOffset();
        VisionUtil.Limelight.configureRobotToCameraOffset();
    }

    public Pose2d getQuestNavCorrectedPose() {
        return VisionUtil.QuestNav.getPose().plus(questNavToPoseEstimateOffset);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getTranslationToSpeaker() {
        Translation2d robotTranslation = getEstimatedPose().getTranslation();
        Translation2d tagTranslation = TagLocation.getSpeakerTagPose().getTranslation();
        if (tagTranslation.getNorm() == 0) { return new Translation2d(); }
        return tagTranslation.minus(robotTranslation);
    }

    public double getAngleToSpeaker() {
        return getTranslationToSpeaker().getAngle().getDegrees();
    }

    public void setPoses(Pose2d pose) {
        this.swerve.resetPose(pose);
        poseEstimator.resetPose(pose);
        setQuestNavPose(pose);
    }

    // TODO SET OFFSET WITH POSE ESTIMATE AS REFERENCE INSTEAD OF LIMELIGHT MEGATAG
    public void setQuestNavOffset() {
        if (VisionUtil.Limelight.isTagClear()) {
            questNavToPoseEstimateOffset = new Transform2d(
                    VisionUtil.Limelight.getMegaTagOnePose().getX(),
                    VisionUtil.Limelight.getMegaTagOnePose().getY(),
                    VisionUtil.Limelight.getMegaTagOnePose().getRotation()
            );
        }
    }

    public void updateQuestNavOffset() {
        if (VisionUtil.Limelight.isTagClear()) {
            // Accumulated error between pose estimator and corrected QuestNav pose
            Transform2d diffMegaTagOneQuest = VisionUtil.Limelight.getMegaTagOnePose().minus(getQuestNavCorrectedPose());
            double dist = diffMegaTagOneQuest.getTranslation().getNorm();
            double rot = diffMegaTagOneQuest.getRotation().getDegrees();
            if (dist > Constants.VisionConstants.UPDATE_QUEST_OFFSET_TRANSLATION_ERROR_THRESHOLD
                    || rot > Constants.VisionConstants.UPDATE_QUEST_OFFSET_ROTATION_ERROR_THRESHOLD) {
                questNavToPoseEstimateOffset = questNavToPoseEstimateOffset.plus(diffMegaTagOneQuest);
            }
        }
    }

    public void setQuestNavPose(Pose2d pose) {
        questNavToPoseEstimateOffset = questNavToPoseEstimateOffset.plus(pose.minus(getQuestNavCorrectedPose()));
    }

    public void updatePoses() {
        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        Optional<EstimatedRobotPose> photonPose = photonPoseEstimator.update(VisionUtil.Photon.BW.latestResult);
        // TODO TEST IF OTHER DOESN'T WORK Pose2d photonPose = VisionUtil.Photon.BW.getPhotonPose();
        Pose2d limelightPose = VisionUtil.Limelight.getMegaTagOnePose();
        if (VisionUtil.Limelight.isTagClear()) {
            poseEstimator.addVisionMeasurement(
                    limelightPose,
                    Timer.getFPGATimestamp() - VisionUtil.Limelight.getLatency()
            );
        }
        if (VisionUtil.Photon.BW.isTagClear() && photonPose.isPresent()) {
            poseEstimator.addVisionMeasurement(
                    photonPose.get().estimatedPose.toPose2d(),
                    photonPose.get().timestampSeconds
                    // TODO TEST IF OTHER DOESN'T WORK VisionUtil.Photon.BW.getTimestamp()
            );
        }
    }

    public void periodic() {
        updatePoses();
        updateQuestNavOffset();
        VisionUtil.Photon.updateResults();
        visionTelemetry.publishValues();
    }
}
