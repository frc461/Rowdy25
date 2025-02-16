package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.PathManager;
import frc.robot.commands.auto.FollowPathRequiringAlgaeCommand;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;

public class DynamicRoutineTest {
    private final AutoEventLooper starter = new AutoEventLooper("DynamicRoutineTest");

    public DynamicRoutineTest(Swerve swerve, Intake intake) {
        AutoTrigger oneStartToSixRight = starter.addTrigger("4,J", () -> new FollowPathRequiringAlgaeCommand(PathManager.FOUR_START_TO_J, false, swerve));
        AutoTrigger sixRightToStation = starter.addTrigger("J,2-station", () -> new FollowPathRequiringAlgaeCommand(PathManager.J_TO_TWO_STATION, false, swerve));
        AutoTrigger findAlgae = starter.addTrigger("FindAlgae", () -> swerve.pathFindFindScoreAlgae(intake::hasAlgae));
        Command stop = Commands.runOnce(swerve::forceStop);

        starter.active().onTrue(oneStartToSixRight.cmd());
        oneStartToSixRight.interrupt().onTrue(findAlgae.cmd()); // Command was interrupted i.e., it couldn't find an object
        oneStartToSixRight.done().onTrue(findAlgae.cmd());

        findAlgae.interrupt().onTrue(stop);
        sixRightToStation.interrupt().onTrue(stop);
    }

    public Command cmd() {
        return starter.cmd();
    }
}
