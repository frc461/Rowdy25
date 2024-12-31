package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class FieldUtil {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.25);

    public enum TagLocation {
        ID_1,
        ID_2,
        ID_3,
        ID_4,
        ID_5,
        ID_6,
        ID_7,
        ID_8,
        ID_9,
        ID_10,
        ID_11,
        ID_12,
        ID_13,
        ID_14,
        ID_15,
        ID_16;

        // TODO UPDATE VALUES FOR 2025
        public static Pose2d getTagLocation(TagLocation tag) {
            return switch (tag) {
                case ID_1 ->
                        new Pose2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(120));
                case ID_2 ->
                        new Pose2d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Rotation2d.fromDegrees(120));
                case ID_3 ->
                        new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Rotation2d.fromDegrees(180));
                case ID_4 ->
                        new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180));
                case ID_5 ->
                        new Pose2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Rotation2d.fromDegrees(-90));
                case ID_6 ->
                        new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Rotation2d.fromDegrees(-90));
                case ID_7 ->
                        new Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0));
                case ID_8 ->
                        new Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Rotation2d.fromDegrees(0));
                case ID_9 ->
                        new Pose2d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Rotation2d.fromDegrees(60));
                case ID_10 ->
                        new Pose2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(60));
                case ID_11 ->
                        new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(-60));
                case ID_12 ->
                        new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Rotation2d.fromDegrees(60));
                case ID_13 ->
                        new Pose2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(180));
                case ID_14 ->
                        new Pose2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(0));
                case ID_15 ->
                        new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Rotation2d.fromDegrees(120));
                case ID_16 ->
                        new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(-120));
            };
        }

        public static Pose2d getTagLocation(double tagID) {
            return switch ((int) tagID) {
                case 1 ->
                        new Pose2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(120));
                case 2 ->
                        new Pose2d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Rotation2d.fromDegrees(120));
                case 3 ->
                        new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Rotation2d.fromDegrees(180));
                case 4 ->
                        new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180)); //+0.0127
                case 5 ->
                        new Pose2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Rotation2d.fromDegrees(-90));
                case 6 ->
                        new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Rotation2d.fromDegrees(-90));
                case 7 ->
                        new Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0)); //+0.0127
                case 8 ->
                        new Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Rotation2d.fromDegrees(0));
                case 9 ->
                        new Pose2d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Rotation2d.fromDegrees(60));
                case 10 ->
                        new Pose2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(60));
                case 11 ->
                        new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(-60));
                case 12 ->
                        new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Rotation2d.fromDegrees(60));
                case 13 ->
                        new Pose2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(180));
                case 14 ->
                        new Pose2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Rotation2d.fromDegrees(0));
                case 15 ->
                        new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Rotation2d.fromDegrees(120));
                case 16 ->
                        new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(-120));
                default -> new Pose2d(0, 0, new Rotation2d());
            };
        }

        public static Pose3d getTagLocation3d(TagLocation tag) {
            return switch (tag) {
                case ID_1 ->
                        new Pose3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(120)));
                case ID_2 ->
                        new Pose3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(120)));
                case ID_3 ->
                        new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(180)));
                case ID_4 ->
                        new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(180)));
                case ID_5 ->
                        new Pose3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(-90)));
                case ID_6 ->
                        new Pose3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(-90)));
                case ID_7 ->
                        new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(0)));
                case ID_8 ->
                        new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(0)));
                case ID_9 ->
                        new Pose3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(60)));
                case ID_10 ->
                        new Pose3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(60)));
                case ID_11 ->
                        new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(-60)));
                case ID_12 ->
                        new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(60)));
                case ID_13 ->
                        new Pose3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(180)));
                case ID_14 ->
                        new Pose3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(0)));
                case ID_15 ->
                        new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(120)));
                case ID_16 ->
                        new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(-120)));
            };
        }

        public static Pose3d getTagLocation3d(double tagID) {
            return switch ((int) tagID) {
                case 1 ->
                        new Pose3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(120)));
                case 2 ->
                        new Pose3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(120)));
                case 3 ->
                        new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(180)));
                case 4 ->
                        new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(180)));
                case 5 ->
                        new Pose3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(-90)));
                case 6 ->
                        new Pose3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(-90)));
                case 7 ->
                        new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(0)));
                case 8 ->
                        new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0, 0, Units.degreesToRadians(0)));
                case 9 ->
                        new Pose3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(60)));
                case 10 ->
                        new Pose3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0, 0, Units.degreesToRadians(60)));
                case 11 ->
                        new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(-60)));
                case 12 ->
                        new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(60)));
                case 13 ->
                        new Pose3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(180)));
                case 14 ->
                        new Pose3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(0)));
                case 15 ->
                        new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(120)));
                case 16 ->
                        new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0, 0, Units.degreesToRadians(-120)));
                default -> new Pose3d(0, 0, 0, new Rotation3d());
            };
        }

        public static Pose2d getSpeakerTagPose() {
            DriverStation.Alliance alliance = Constants.ALLIANCE_SUPPLIER.get();
            return alliance == null ? new Pose2d() : getTagLocation(
                    alliance == DriverStation.Alliance.Blue
                            ? TagLocation.ID_7
                            : TagLocation.ID_4
            );
        }
    }
}
