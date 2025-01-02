package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.PathManager;
import frc.robot.commands.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoTrigger starter = new AutoTrigger("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoTrigger testPath = new AutoTrigger("TestPath", new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));

        starter.active().onTrue(testPath.cmd());
    }

    public Command cmd() {
        return starter.cmd();
    }
}
