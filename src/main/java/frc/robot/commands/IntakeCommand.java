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
        switch (intake.getCurrentState()) {
            case INTAKE:
                if (intake.hasAlgae()) {
                    intake.setState(Intake.States.HAS_ALGAE);
                } else if (intake.hasCoral()) {
                    System.out.println("HAS CORAL!!!!!!!");
                    intake.setState(Intake.States.IDLE);
                } else {
                    intake.setIntakeSpeed(0.75);
                }
            case OUTTAKE:
                if (!intake.hasAlgae() && !intake.hasCoral()) {
                    intake.setState(Intake.States.IDLE);
                } else {
                    intake.setIntakeSpeed(-0.5);
                }
            case HAS_ALGAE:
                if (!intake.hasAlgae()) {
                    intake.setState(Intake.States.IDLE);
                } else {
                    intake.pulseIntake();
                }
            case IDLE:
                if (intake.hasCoral() || intake.hasAlgae()) {
                    intake.setState(intake.hasAlgae() ? Intake.States.HAS_ALGAE : Intake.States.IDLE);
                } else {
                    intake.setIntakeSpeed(0.0);
                }
        }
    }
}
