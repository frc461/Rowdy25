package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Lights;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    public enum State {
        IDLE,
        HAS_ALGAE,
        INTAKE,
        INTAKE_SLOW,
        INTAKE_OUT,
        INTAKE_OVERRIDE,
        OUTTAKE,
        OUTTAKE_SLOW,
        OUTTAKE_L1
    }

    public enum StallIntent {
        CORAL_STUCK,
        HAS_ALGAE
    }

    private State currentState;

    private final TalonFX intake;
    private final DigitalInput beamBreak;

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    private StallIntent stallIntent = StallIntent.CORAL_STUCK;
    public Trigger hasAlgaeOrCoralStuck;

    public Intake() {
        intake = new TalonFX(Constants.IntakeConstants.MOTOR_ID);

        intake.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.IntakeConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        beamBreak = new DigitalInput(Constants.IntakeConstants.BEAMBREAK_DIO_PORT);
        currentState = State.IDLE;

        hasAlgaeOrCoralStuck = new Trigger(() -> intake.getStatorCurrent().getValueAsDouble() > 20.0).debounce(0.5); // TODO SHOP: TEST THIS
    }

    public double getCurrent() {
        return intake.getStatorCurrent().getValueAsDouble();
    }

    public State getState() {
        return currentState;
    }

    public boolean hasCoral() {
        return !beamBreak.get();
    }

    public boolean coralStuck() {
        return hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.CORAL_STUCK; // TODO SHOP: TEST THIS
    }

    public boolean hasAlgae() {
        return hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.HAS_ALGAE; // TODO SHOP: TEST THIS
    }

    public boolean atIdleState() {
        return currentState == State.IDLE;
    }

    public boolean atHasAlgaeState() {
        return currentState == State.HAS_ALGAE;
    }

    private void setState(State newState) {
        currentState = newState;
    }

    public void setIdleState() {
        if (hasAlgae()) {
            setState(State.HAS_ALGAE);
        } else {
            setState(State.IDLE);
        }
    }

    public void setAlgaeIntakeState() {
        stallIntent = StallIntent.HAS_ALGAE;
        setIntakeState(false);
    }

    public void setCoralIntakeState() {
        stallIntent = StallIntent.CORAL_STUCK;
        setIntakeState(false);
    }

    public void setIntakeState(boolean override) {
        if (override) {
            stallIntent = StallIntent.CORAL_STUCK;
            setState(State.INTAKE_OVERRIDE);
        } else {
            setState(State.INTAKE);
        }
    }

    public void setIntakeOutState() {
        setState(State.INTAKE_OUT);
    }

    public void setOuttakeState() {
        setState(State.OUTTAKE);
    }

    public void setOuttakeL1State() {
        setState(State.OUTTAKE_L1);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    @Override
    public void periodic() {
        intakeTelemetry.publishValues();

        Lights.setLights(hasCoral() || hasAlgae());
    }
}
