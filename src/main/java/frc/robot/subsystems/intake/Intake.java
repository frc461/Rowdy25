package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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

    private State currentState;

    private final TalonFX intake;
    private final DigitalInput beamBreak;
    private final Timer pulseTimer = new Timer();

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    public Trigger hasAlgae;

    public Intake() {
        intake = new TalonFX(Constants.IntakeConstants.LEAD_ID);

        intake.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.IntakeConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        beamBreak = new DigitalInput(Constants.IntakeConstants.BEAMBREAK_ID);
        currentState = State.IDLE;
        pulseTimer.start();

        hasAlgae = new Trigger(() -> intake.getStatorCurrent().getValueAsDouble() > 40.0).debounce(0.5); // TODO SHOP: TEST THIS
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

    public boolean hasAlgae() {
        return hasAlgae.getAsBoolean() || currentState == State.HAS_ALGAE; // TODO SHOP: TEST THIS
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

    public void setIntakeState() {
        setIntakeState(false);
    }

    public void setIntakeState(boolean override) {
        if (override) {
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

    public void pulseIntake() { // TODO SHOP: TEST THIS WITH GROUND ALGAE INTAKE
        if ((int) pulseTimer.get() % 2 == 0) {
            setIntakeSpeed(0.1);
        } else {
            setIntakeSpeed(0.0);
        }
    }

    @Override
    public void periodic() {
        intakeTelemetry.publishValues();

        Lights.setLights(hasCoral() || hasAlgae());
    }
}
