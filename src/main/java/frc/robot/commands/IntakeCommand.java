package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        switch (intake.getState()) {
            case INTAKE:
                intake.setIntakeSpeed(0.3);
                break;
            case INTAKE_OUT:
                intake.setIntakeSpeed(0.5);
                break;
            case OUTTAKE:
                intake.setIntakeSpeed(-0.2);
                break;
            case HAS_ALGAE:
                intake.pulseIntake();
                break;
            case IDLE:
                intake.setIntakeSpeed(0.0);
                break;
        }
    }
}
