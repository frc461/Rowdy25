package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Lights;

import frc.robot.constants.Constants;

import java.util.function.DoubleConsumer;

public class Intake extends SubsystemBase {
    public enum State {
        IDLE,
        HAS_ALGAE,
        INTAKE,
        INTAKE_OUT,
        INTAKE_OVERRIDE,
        OUTTAKE,
        OUTTAKE_L1
    }

    private State currentState;

    private final TalonFX intake;
    private final DigitalInput beamBreak;
    private final Timer pulseTimer = new Timer();

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    private double proximityObjectDetectionThreshold = Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD;
    public DoubleConsumer setProximityObjectDetectionThreshold = threshold -> proximityObjectDetectionThreshold = threshold;

    public Intake() { // TODO: IMPLEMENT STALL DETECTION (HIGH AMPAGE FOR AN EXTENDED PERIOD OF TIME)
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

        try (TalonFX intake2 = new TalonFX(Constants.IntakeConstants.FOLLOWER_ID)) {
            intake2.setControl(new Follower(Constants.IntakeConstants.LEAD_ID, false));
        }

        beamBreak = new DigitalInput(Constants.IntakeConstants.BEAMBREAK_ID);
        currentState = State.IDLE;
        pulseTimer.start();
    }

    public State getState() {
        return currentState;
    }

    public boolean beamBreakBroken() {
        return !beamBreak.get();
    }

    public boolean hasCoral() {
        return false;
    }

    public boolean hasAlgae() {
        return false; // TODO SHOP: TUNE THIS
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

    public void pulseIntake() {
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
