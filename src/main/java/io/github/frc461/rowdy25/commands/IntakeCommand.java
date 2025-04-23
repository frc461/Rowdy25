package io.github.frc461.rowdy25.commands;

import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.subsystems.intake.Intake;

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
                if (intake.hasCoral() || intake.algaeStuck()) {
                    intake.setIdleState();
                } else if (intake.coralEntered() && !intake.beamBreakBroken()) {
                    intake.setIntakeSlowState();
                } else if (intake.beamBreakBroken() && !intake.coralEntered()) {
                    intake.setOuttakeSlowState();
                } else {
                    intake.setIntakeSpeed(0.45);
                }
                break;
            case INTAKE_SLOW:
                if (intake.hasCoral() || intake.algaeStuck()) {
                    intake.setIdleState();
                } else if (intake.coralEntered() && !intake.beamBreakBroken()) {
                    intake.setIntakeSpeed(0.15);
                } else if (intake.beamBreakBroken() && !intake.coralEntered()) {
                    intake.setOuttakeSlowState();
                } else {
                    intake.setIntakeState(false);
                }
                break;
            case INTAKE_OUT:
                intake.setIntakeSpeed(0.65);
                break;
            case INTAKE_OVERRIDE:
                intake.setIntakeSpeed(0.35);
                break;
            case OUTTAKE:
                intake.setIntakeSpeed(-0.5);
                break;
            case OUTTAKE_SLOW:
                if (intake.hasCoral() || intake.algaeStuck()) {
                    intake.setIdleState();
                } else if (intake.coralEntered() && !intake.beamBreakBroken()) {
                    intake.setIntakeSlowState();
                } else if (intake.beamBreakBroken() && !intake.coralEntered()) {
                    intake.setIntakeSpeed(-0.15);
                } else {
                    intake.setIntakeState(false);
                }
                break;
            case OUTTAKE_L1:
                intake.setIntakeSpeed(-0.4);
                break;
            case HAS_ALGAE:
                intake.setIntakeSpeed(0.03); // TODO SHOP: TEST THIS
                break;
            case IDLE:
                intake.setIntakeSpeed(0.0);
                break;
        }
    }
}
