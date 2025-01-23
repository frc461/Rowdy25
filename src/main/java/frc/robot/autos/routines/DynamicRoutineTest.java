package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.auto.FollowPathRequiringAlgaeCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoTrigger testPath = starter.addTrigger("TestPath", new FollowPathRequiringAlgaeCommand(PathManager.ONE_START_TO_SIX_RIGHT, false, swerve));
        AutoTrigger testPath2 = starter.addTrigger("TestPath2", new FollowPathRequiringAlgaeCommand(PathManager.SIX_RIGHT_TO_STATION, false, swerve));
        AutoTrigger findObject2 = starter.addTrigger("FindObject2", swerve.pathFindFindScoreAlgae());
        Command stop = Commands.runOnce(swerve::forceStop);

        starter.active().onTrue(testPath.cmd());
        testPath.interrupt().onTrue(findObject2.cmd()); // Command was interrupted i.e., it couldn't find an object
        testPath.done().onTrue(testPath2.cmd());

        findObject2.interrupt().onTrue(stop);
        testPath2.interrupt().onTrue(stop);
    }

    public Command cmd() {
        return starter.cmd();
    }
}
