package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorPeriod;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Lights;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    public enum State {
        IDLE,
        HAS_ALGAE,
        INTAKE,
        INTAKE_OUT,
        OUTTAKE
    }

    private State currentState;

    private final TalonFX motor;
    private final Canandcolor canandcolor;
    private final DigitalInput beamBreak;
    private final Timer pulseTimer = new Timer();

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

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
        beamBreak = new DigitalInput(Constants.IntakeConstants.BEAMBREAK_ID);
        currentState = State.IDLE;
        pulseTimer.start();
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

    public boolean hasCoral() {
        return !beamBreak.get();
    }

    public boolean hasAlgae() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kAqua); // TODO SHOP: TUNE THIS
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
        setState(State.INTAKE);
    }

    public void setIntakeOutState() {
        setState(State.INTAKE_OUT);
    }

    public void setOuttakeState() {
        setState(State.OUTTAKE);
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
