package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public record EstimatedRobotPose(Pose3d estimatedPose, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed,
                                 Matrix<N3, N1> stdDevs) {
}
