package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.PathManager;
import frc.robot.commands.FollowPathDynamicCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicRoutineTest {
    private final AutoRoutine routine = new AutoRoutine("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve) {

        routine.active().onTrue()

        new FollowPathDynamicCommand(PathManager.TEST_PATH, false, swerve);
    }

    public Command cmd() {

    }
}
