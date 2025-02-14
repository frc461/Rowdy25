package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
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
    private final Canandcolor canandcolor; // TODO SHOP: TUNE
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
        canandcolor.setLampLEDBrightness(1.0);
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
        return getProximity() < 0.05;
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
