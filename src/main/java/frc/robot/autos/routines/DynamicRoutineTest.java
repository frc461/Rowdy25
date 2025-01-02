package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.auto.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoTrigger testPath = starter.addTrigger("TestPath", new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));
        AutoTrigger testPath2 = starter.addTrigger("TestPath2", new FollowPathDynamicCommand(PathManager.TEST_PATH_2, false, swerve));
        AutoTrigger findNote2 = starter.addTrigger("FindNote2", swerve.pathFindFindScoreObject());
        Command stop = Commands.runOnce(swerve::forceStop);

        starter.active().onTrue(testPath.cmd());
        starter.active().onTrue(Commands.run(() -> System.out.println(testPath.interrupt().getAsBoolean())));
        testPath.interrupt().onTrue(findNote2.cmd()); // Command was interrupted i.e., it couldn't find a note
        testPath.interrupt().onTrue(Commands.runOnce(
                () -> System.out.println("This worked")
        ));
        testPath.done().onTrue(testPath2.cmd());
        testPath.done().onTrue(Commands.runOnce(
                () -> System.out.println("This worked 3")
        ));

        findNote2.interrupt().onTrue(stop);
        testPath2.interrupt().onTrue(stop);

    }

    public Command cmd() {
        return starter.cmd();
    }
}
