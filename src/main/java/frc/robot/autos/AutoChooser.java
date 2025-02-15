package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.autos.routines.DynamicRoutineTest;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.intake.Intake;

public final class AutoChooser {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoChooser(Swerve swerve, Intake intake) {
        autoChooser.addOption("DynamicAutoTest", new DynamicRoutineTest(swerve, intake).cmd());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command get() {
        return autoChooser.getSelected();
    }
}
