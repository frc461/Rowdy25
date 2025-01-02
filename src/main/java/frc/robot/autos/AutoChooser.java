package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.routines.DynamicRoutineTest;
import frc.robot.subsystems.drivetrain.Swerve;

public final class AutoChooser {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoChooser(Swerve swerve) {
        autoChooser.addOption("DynamicAutoTest", new DynamicRoutineTest(swerve).cmd());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command get() {
        return autoChooser.getSelected();
    }

    public Command pathFindToPose(Pose2d targetPose) {
         PathConstraints constraints = new PathConstraints(
                Constants.MAX_VEL,
                Constants.MAX_ACCEL,
                Constants.MAX_DESIRED_ANGULAR_VEL,
                Constants.MAX_ANGULAR_ACCEL
        );
         
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
        );
    }
}
