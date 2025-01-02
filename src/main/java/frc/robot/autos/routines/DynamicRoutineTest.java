package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoPathTrigger testPath = starter.addPath(new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));

        starter.active().onTrue(testPath.cmd());
        testPath.inactive().onTrue(Commands.runOnce(starter::kill));
    }

    public Command cmd() {
        return starter.cmd();
    }
}
