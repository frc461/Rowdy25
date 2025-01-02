package frc.robot.autos.routines;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {
        AutoPathTrigger testPath = starter.addPath(new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));
        AutoPathTrigger testPath2 = starter.addPath(new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve));
        Command stop = Commands.runOnce(swerve::forceStop);

        starter.active().onTrue(testPath.cmd());
        testPath.inactive().and(testPath.done().negate()).onTrue(stop); // Command was interrupted i.e., it couldn't find a note, TODO IMPLEMENT LOOK FOR NOTE
        testPath.done().onTrue(testPath2.cmd());

        testPath2.inactive().and(testPath2.done().negate()).onTrue(stop);
    }

    public Command cmd() {
        return starter.cmd();
    }
}
