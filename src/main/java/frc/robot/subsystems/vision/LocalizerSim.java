package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.FieldUtil;
import frc.robot.util.vision.PhotonUtil;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class LocalizerSim {
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    public LocalizerSim() {
        visionSim.addAprilTags(FieldUtil.layout2025);

        SimCameraProperties camProperties = new SimCameraProperties();
        camProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(70.0));
        camProperties.setCalibError(0.25, 0.08);
        camProperties.setFPS(30.0);
        camProperties.setAvgLatencyMs(25.0);
        camProperties.setLatencyStdDevMs(10.0);

        PhotonCameraSim BWTopRightSim = new PhotonCameraSim(PhotonUtil.BW.getCamera(PhotonUtil.BW.BWCamera.TOP_RIGHT), camProperties);
        PhotonCameraSim BWTopLeftSim = new PhotonCameraSim(PhotonUtil.BW.getCamera(PhotonUtil.BW.BWCamera.TOP_LEFT), camProperties);
        PhotonCameraSim BWBackSim = new PhotonCameraSim(PhotonUtil.BW.getCamera(PhotonUtil.BW.BWCamera.BACK), camProperties);

        visionSim.addCamera(BWTopRightSim, PhotonUtil.BW.getRobotToBWOffset(PhotonUtil.BW.BWCamera.TOP_RIGHT));
        visionSim.addCamera(BWTopLeftSim, PhotonUtil.BW.getRobotToBWOffset(PhotonUtil.BW.BWCamera.TOP_LEFT));
        visionSim.addCamera(BWBackSim, PhotonUtil.BW.getRobotToBWOffset(PhotonUtil.BW.BWCamera.BACK));
    }

    public void update(Pose2d strategyPose) {
        visionSim.update(strategyPose);
    }
}
