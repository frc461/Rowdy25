package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
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
        OUTTAKE
    }

    private final TalonFX motor;
    private final Canandcolor canandcolor; // TODO SHOP: Use https://docs.reduxrobotics.com/alchemist/ to configure IDs
    private final Timer pulseTimer = new Timer();
    private State currentState;

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    public Intake() {
        motor = new TalonFX(Constants.IntakeConstants.MOTOR_ID);

        motor.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(Constants.IntakeConstants.PEAK_VOLTAGE))
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

    public State getCurrentState() {
        return currentState;
    }

    public double[] getColorReading() {
        return new double[] { canandcolor.getBlue(), canandcolor.getGreen(), canandcolor.getRed() };
    }

    public double getProximity() {
        return canandcolor.getProximity();
    }
 
    public boolean hasCoral() {
        return getProximity() < 0.1; // TODO SHOP: Set the color to close to the color of coral
    }

    public boolean hasAlgae() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kAqua); // TODO SHOP: Set the color to close to the color of algae
    }

    public void toggleIntakeState() {
        setState(currentState == State.INTAKE ? State.IDLE : State.INTAKE);
    }

    public void toggleOuttakeState() {
        setState(currentState == State.OUTTAKE ? State.IDLE : State.OUTTAKE);
    }

    public void setState(State state) {
        currentState = state;
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

        if (hasCoral() || hasAlgae()) {
            Lights.setLights(true);
        } else {
            Lights.setLights(false);
        }

        switch (getCurrentState()) {
            case INTAKE:
                if (hasAlgae()) {
                    setState(Intake.State.HAS_ALGAE);
                } else if (hasCoral()) {
                    setState(Intake.State.IDLE);
                } else {
                    setIntakeSpeed(0.5);
                }
                break;
            case OUTTAKE:
                if (!hasAlgae() && !hasCoral()) {
                    setState(Intake.State.IDLE);
                } else {
                    setIntakeSpeed(-0.5);
                }
                break;
            case HAS_ALGAE:
                if (!hasAlgae()) {
                    setState(Intake.State.IDLE);
                } else {
                    pulseIntake();
                }
                break;
            case IDLE:
                if (hasCoral() || hasAlgae()) {
                    setState(hasAlgae() ? Intake.State.HAS_ALGAE : Intake.State.IDLE);
                } else {
                    setIntakeSpeed(0.0);
                }
                break;
        }
    }
}
