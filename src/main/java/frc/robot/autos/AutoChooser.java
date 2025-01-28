package frc.robot.autos;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.FieldUtil;

public final class AutoChooser {
    public enum SidePriority {
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX;
    }

    public enum LevelPriority {
        L1,
        L2,
        L3,
        L4;
    }

    public enum Stage {
        SCORE,
        PICKUP;
    }

    public SidePriority sidePriority;
    public LevelPriority levelPriority;
    public Stage currentStage = Stage.SCORE;

    private final SendableChooser<SidePriority> reefSideChooser = new SendableChooser<>();
    private final SendableChooser<LevelPriority> reefLevelChooser = new SendableChooser<>();

    public AutoChooser(Swerve swerve) {
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
        sidePriority = reefSideChooser.getSelected();
        levelPriority = reefLevelChooser.getSelected();

        AutoEventLooper starter = new AutoEventLooper("Dynamic Auto");

        starter.active().onTrue(generatePathCommand(poseSupplier.get(), sidePriority, levelPriority));
        

        return Commands.none();
    }

    // TODO FIX THIS SHIT
    public Command generatePathCommand(Pose2d currentPose, SidePriority sidePriority, LevelPriority levelPriority) {
        Pose2d targetPose;
        if (currentStage.equals(Stage.SCORE)) {
            targetPose = FieldUtil.Coral.getNearestBranchPose(currentPose);
        } else {
            targetPose = FieldUtil.Coral.getNearestCoralStationTagPose(currentPose);
        }
        return PathManager.pathFindToPose(targetPose);
    }
}
