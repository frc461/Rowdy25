package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorPeriod;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Lights;

import frc.robot.constants.Constants;

import java.util.function.DoubleConsumer;

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

    private final TalonFX motor;
    private final Canandcolor canandcolor;
    private final DigitalInput beamBreak;
    private final Timer pulseTimer = new Timer();

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    public Trigger coralStuck;
    private double proximityObjectDetectionThreshold = Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD;
    public DoubleConsumer setProximityObjectDetectionThreshold = threshold -> proximityObjectDetectionThreshold = threshold;

    public Intake() {
        motor = new TalonFX(Constants.IntakeConstants.MOTOR_ID);

        motor.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.IntakeConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        CanandEventLoop.getInstance();
        canandcolor = new Canandcolor(Constants.IntakeConstants.SENSOR_ID);
        canandcolor.setSettings(
                canandcolor.getSettings()
                        .setAlignProximityFramesToIntegrationPeriod(true)
                        .setProximityIntegrationPeriod(ProximityPeriod.k5ms)
                        .setAlignColorFramesToIntegrationPeriod(true)
                        .setColorIntegrationPeriod(ColorPeriod.k25ms)
                        .setDigoutFramePeriod(0.02)
        );
        canandcolor.setLampLEDBrightness(0.0);
        beamBreak = new DigitalInput(Constants.IntakeConstants.BEAMBREAK_DIO_PORT);
        currentState = State.IDLE;
        pulseTimer.start();

        coralStuck = new Trigger(this::atIntakeSlowState).debounce(1.5) // TODO SHOP: TEST THIS
                .or(new Trigger(() -> motor.getStatorCurrent().getValueAsDouble() > 40.0).debounce(0.5));
    }

    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public State getState() {
        return currentState;
    }

    public double[] getColorReading() {
        return new double[] { canandcolor.getBlue(), canandcolor.getGreen(), canandcolor.getRed() };
    }

    public double getProximity() {
        return canandcolor.getProximity();
    }

    public boolean beamBreakBroken() {
        return !beamBreak.get();
    }

    public boolean coralEntered() {
        return getProximity() < proximityObjectDetectionThreshold;
    }

    public boolean barelyHasCoral() {
        return beamBreakBroken() || coralEntered();
    }

    public boolean hasCoral() {
        return beamBreakBroken() && coralEntered();
    }

    public boolean hasAlgae() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kAqua); // TODO SHOP: TUNE THIS
    }

    public boolean atIdleState() {
        return currentState == State.IDLE;
    }

    public boolean atIntakeSlowState() {
        return currentState == State.INTAKE_SLOW;
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

    public void setIntakeSlowState() {
        setState(State.INTAKE_SLOW);
    }

    public void setIntakeOutState() {
        setState(State.INTAKE_OUT);
    }

    public void setOuttakeState() {
        setState(State.OUTTAKE);
    }

    public void setOuttakeSlowState() {
        setState(State.OUTTAKE_SLOW);
    }

    public void setOuttakeL1State() {
        setState(State.OUTTAKE_L1);
    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
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
