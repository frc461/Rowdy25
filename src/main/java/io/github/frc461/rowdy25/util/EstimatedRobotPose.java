package io.github.frc461.rowdy25.util;

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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

/**
 * A record representing an estimated robot pose along with associated metadata.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @param estimatedPose   The estimated pose of the robot in 3D space.
 * @param timestampSeconds The timestamp (in seconds) when the pose was estimated.
 * @param targetsUsed     A list of PhotonTrackedTarget objects that were used to estimate the pose.
 * @param stdDevs         A 3x1 matrix representing the standard deviations of the pose estimation in x, y, and z directions.
 */
public record EstimatedRobotPose(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed,
                                 Matrix<N3, N1> stdDevs) {
}
