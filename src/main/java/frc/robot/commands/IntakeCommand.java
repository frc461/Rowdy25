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
                if (intake.hasAlgae()) {
                    intake.setHasAlgaeState();
                } else if (intake.hasCoral()) {
                    intake.setIdleState();
                } else {
                    intake.setIntakeSpeed(0.1);
                }
                break;
            case INTAKE_OUT:
                if (!intake.hasCoral() && !intake.hasCoral()) {
                    intake.setIdleState();
                } else {
                    intake.setIntakeSpeed(0.5);
                }
                break;
            case OUTTAKE:
                if (!intake.hasAlgae() && !intake.hasCoral()) {
                    intake.setIdleState();
                } else {
                    intake.setIntakeSpeed(-0.5);
                }
                break;
            case HAS_ALGAE:
                if (!intake.hasAlgae()) {
                    intake.setIdleState();
                } else {
                    intake.pulseIntake();
                }
                break;
            case IDLE:
                if (intake.hasAlgae()) {
                    intake.setHasAlgaeState();
                } else {
                    intake.setIntakeSpeed(0.0);
                }
                break;
        }
    }
}
