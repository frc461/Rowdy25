package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.VisionUtil;

public final class Auto {
  
    private AutoFactory autoFactory;

    public Auto(AutoFactory autoFactory) {
        this.autoFactory = autoFactory;
    }

    public Command rightThreePiece() {
        AutoRoutine autoRoutine = autoFactory.newRoutine("2,5Ce,4Ce");
        AutoTrajectory start = autoRoutine.trajectory("2,5Ce");
        AutoTrajectory fiveExists = autoRoutine.trajectory("5Ce,shoot");
        AutoTrajectory fiveDoesNotExist = autoRoutine.trajectory("5Ce,4Ce");
        AutoTrajectory getFour = autoRoutine.trajectory("shoot,4Ce");

        autoRoutine.active().onTrue(
                Commands.sequence(
                    autoRoutine.resetOdometry(start),
                    start.cmd()
                    )
                );

        start.done().and(VisionUtil.Photon.Color::hasTargets).onTrue(fiveExists.cmd());
        start.done().and(() -> !VisionUtil.Photon.Color.hasTargets()).onTrue(fiveDoesNotExist.cmd());

        fiveExists.done().onTrue(getFour.cmd());

        return autoRoutine.cmd();
    }

    public Command branchingTest() {
        AutoRoutine autoRoutine = autoFactory.newRoutine("branchingTest");
        AutoTrajectory start = autoRoutine.trajectory("start");
        AutoTrajectory end1 = autoRoutine.trajectory("end1");
        AutoTrajectory end2 = autoRoutine.trajectory("end2");

        autoRoutine.active().onTrue(
                Commands.sequence(
                    autoRoutine.resetOdometry(start),
                    start.cmd()
                    )
                );

        start.done().and(VisionUtil.Photon.Color::hasTargets).onTrue(end1.cmd());
        start.done().and(() -> !VisionUtil.Photon.Color.hasTargets()).onTrue(end2.cmd());

        return autoRoutine.cmd();
    }
}
