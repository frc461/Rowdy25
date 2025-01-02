package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.auto.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoDynamicPathTrigger testPath = starter.addDynamicPath("TestPath", new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));
        AutoDynamicPathTrigger testPath2 = starter.addDynamicPath("TestPath2", new FollowPathDynamicCommand(PathManager.TEST_PATH_2, false, swerve));
        AutoTrigger findNote2 = starter.addTrigger("FindNote2", swerve.pathFindFindScoreObject());
        Command stop = Commands.runOnce(swerve::forceStop);

        starter.active().onTrue(testPath.cmd());
        testPath.interrupt().onTrue(findNote2.cmd()); // Command was interrupted i.e., it couldn't find a note
        testPath.done().onTrue(testPath2.cmd());

        findNote2.interrupt().onTrue(stop);
        testPath2.interrupt().onTrue(stop);

    }

    public Command cmd() {
        return starter.cmd();
    }
}
