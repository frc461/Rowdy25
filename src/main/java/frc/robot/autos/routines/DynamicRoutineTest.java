package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.PathManager;
import frc.robot.commands.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest extends SequentialCommandGroup {
    public DynamicRoutineTest(Swerve swerve) {
        setName("DynamicAutoTest");
        addRequirements(swerve);
        addCommands(new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));
    }
}
