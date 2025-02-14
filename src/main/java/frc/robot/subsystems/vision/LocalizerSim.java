package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.FieldUtil;
import frc.robot.util.VisionUtil;
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

        PhotonCameraSim BWTopRightSim = new PhotonCameraSim(VisionUtil.Photon.BW.getCamera(VisionUtil.Photon.BW.BWCamera.TOP_RIGHT), camProperties);
        PhotonCameraSim BWTopLeftSim = new PhotonCameraSim(VisionUtil.Photon.BW.getCamera(VisionUtil.Photon.BW.BWCamera.TOP_LEFT), camProperties);
        PhotonCameraSim BWBackSim = new PhotonCameraSim(VisionUtil.Photon.BW.getCamera(VisionUtil.Photon.BW.BWCamera.BACK), camProperties);

        visionSim.addCamera(BWTopRightSim, VisionUtil.Photon.BW.getRobotToBWOffset(VisionUtil.Photon.BW.BWCamera.TOP_RIGHT));
        visionSim.addCamera(BWTopLeftSim, VisionUtil.Photon.BW.getRobotToBWOffset(VisionUtil.Photon.BW.BWCamera.TOP_LEFT));
        visionSim.addCamera(BWBackSim, VisionUtil.Photon.BW.getRobotToBWOffset(VisionUtil.Photon.BW.BWCamera.BACK));
    }

    public void update(Pose2d strategyPose) {
        visionSim.update(strategyPose);
    }
}
