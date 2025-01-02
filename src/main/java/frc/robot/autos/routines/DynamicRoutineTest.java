package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.auto.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoTrigger testPath = starter.addTrigger("TestPath", new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));
        AutoTrigger testPath2 = starter.addTrigger("TestPath2", new FollowPathDynamicCommand(PathManager.TEST_PATH_2, false, swerve));
        Command stop = Commands.runOnce(swerve::forceStop);

        starter.active().onTrue(testPath.cmd());
        testPath.interrupt().onTrue(stop); // Command was interrupted i.e., it couldn't find a note, TODO IMPLEMENT LOOK FOR NOTE

        testPath.done().onTrue(testPath2.cmd());

        testPath2.interrupt().onTrue(stop);
    }

    public Command cmd() {
        return starter.cmd();
    }
}
