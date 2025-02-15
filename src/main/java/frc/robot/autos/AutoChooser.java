package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;

public final class AutoChooser {
    // TODO: CONFIGURE THESE VALUES, ADD ANOTHER CHOOSER
    public enum StartPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum SidePriority {
        ONE(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FieldUtil.AprilTag.ID_7 : FieldUtil.AprilTag.ID_18),
        TWO(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FieldUtil.AprilTag.ID_6 : FieldUtil.AprilTag.ID_19),
        THREE(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FieldUtil.AprilTag.ID_11 : FieldUtil.AprilTag.ID_20),
        FOUR(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FieldUtil.AprilTag.ID_10 : FieldUtil.AprilTag.ID_21),
        FIVE(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FieldUtil.AprilTag.ID_9 : FieldUtil.AprilTag.ID_22),
        SIX(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red ? FieldUtil.AprilTag.ID_8 : FieldUtil.AprilTag.ID_17),;

        public final FieldUtil.AprilTag tag;

        SidePriority(FieldUtil.AprilTag tag) {
            this.tag = tag;
        }
    }

    public enum LevelPriority {
        L1,
        L2,
        L3,
        L4;
    }

    public StartPosition startPosition;
    public SidePriority sidePriority;
    public LevelPriority levelPriority;

    private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
    private final SendableChooser<SidePriority> reefSideChooser = new SendableChooser<>();
    private final SendableChooser<LevelPriority> reefLevelChooser = new SendableChooser<>();

    public AutoChooser() {
        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);

        for (SidePriority priority : SidePriority.values()) {
            reefSideChooser.addOption(priority.name(), priority);
        }
        SmartDashboard.putData("Reef Side Priority", reefSideChooser);

        for(LevelPriority priority : LevelPriority.values()) {
            reefLevelChooser.addOption(priority.name(), priority);
        }
        SmartDashboard.putData("Reef Level Priority", reefLevelChooser);
    }


    public Command getFinalAutoCommand(Supplier<Pose2d> poseSupplier) {
        startPosition = startPositionChooser.getSelected();
        sidePriority = reefSideChooser.getSelected();
        levelPriority = reefLevelChooser.getSelected();

        return PathManager.generateAutoEventLooper(startPosition, sidePriority, levelPriority).cmd();
    }
}
