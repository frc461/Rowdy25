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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import io.github.frc461.rowdy25.util.FieldUtil;
import io.github.frc461.rowdy25.util.vision.PhotonUtil;
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

        PhotonCameraSim BWTopRightSim = new PhotonCameraSim(PhotonUtil.BW.BWCamera.TOP_RIGHT.getCamera(), camProperties);
        PhotonCameraSim BWTopLeftSim = new PhotonCameraSim(PhotonUtil.BW.BWCamera.TOP_LEFT.getCamera(), camProperties);
        PhotonCameraSim BWBackSim = new PhotonCameraSim(PhotonUtil.BW.BWCamera.BACK.getCamera(), camProperties);

        visionSim.addCamera(BWTopRightSim, PhotonUtil.BW.BWCamera.TOP_RIGHT.getRobotToCameraOffset());
        visionSim.addCamera(BWTopLeftSim, PhotonUtil.BW.BWCamera.TOP_LEFT.getRobotToCameraOffset());
        visionSim.addCamera(BWBackSim, PhotonUtil.BW.BWCamera.BACK.getRobotToCameraOffset());
    }

    public void update(Pose2d strategyPose) {
        visionSim.update(strategyPose);
    }
}
