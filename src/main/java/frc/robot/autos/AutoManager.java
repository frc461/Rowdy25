package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.VisionUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public final class AutoManager {
  
    private final AutoFactory autoFactory;
    public Map<String, Supplier<AutoRoutine>> allRoutines = new HashMap<>();

    public AutoManager(AutoFactory autoFactory) {
        this.autoFactory = autoFactory;
        addAllAutos();
    }

    public void addAllAutos() {
        rightThreePiece();
        branchingTest();
    }

    public void rightThreePiece() {
        String name = "2,5Ce,4Ce";
        AutoRoutine autoRoutine = autoFactory.newRoutine(name);
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

        allRoutines.put(name, () -> autoRoutine);
    }

    public void branchingTest() {
        String name = "branchingTest";
        AutoRoutine autoRoutine = autoFactory.newRoutine(name);
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

        allRoutines.put(name, () -> autoRoutine);
    }
}
