package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Lights;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private enum States {
        IDLE,
        HAS_ALGAE,
        INTAKE,
        OUTTAKE
    }

    private final TalonFX motor;
    private final Canandcolor canandcolor; // TODO SHOP: Use https://docs.reduxrobotics.com/alchemist/ to configure IDs
    private final CanandcolorSettings currentCanandcolorSettings;
    private States currentState;
    private boolean stateChanged;
    private final Timer pulseTimer = new Timer();

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    public Intake() {
        motor = new TalonFX(Constants.IntakeConstants.MOTOR_ID);

        motor.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        CanandEventLoop.getInstance();
        canandcolor = new Canandcolor(Constants.IntakeConstants.SENSOR_ID);
        currentCanandcolorSettings = canandcolor.getSettings();
        currentState = States.IDLE;
        pulseTimer.start();
    }

    public double[] getColorReading() {
        return new double[] { canandcolor.getBlue(), canandcolor.getGreen(), canandcolor.getRed() };
    }
 
    public boolean hasCoral() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kWhiteSmoke); // TODO SHOP: Set the color to close to the color of coral
    }

    public boolean hasAlgae() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kAqua); // TODO SHOP: Set the color to close to the color of algae
    }

    public Command intake() {
        return runOnce(() -> setState(currentState == States.INTAKE ? States.IDLE : States.INTAKE));
    }

    public Command outtake() {
        return runOnce(() -> setState(currentState == States.OUTTAKE ? States.IDLE : States.OUTTAKE));
    }

    private void setState(States state) {
        currentState = state;
        stateChanged = true;
    }

    private void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    private void pulseIntake() {
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

        if (stateChanged) {
            switch (currentState) {
                case IDLE, HAS_ALGAE:
                    setIntakeSpeed(0.0);
                case INTAKE:
                    setIntakeSpeed(0.75);
                case OUTTAKE:
                    setIntakeSpeed(-0.5);
            }
            stateChanged = false;
        } else {
            switch (currentState) {
                case INTAKE:
                    if (hasAlgae()) {
                        setState(States.HAS_ALGAE);
                        canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(1.0));
                    } else if (hasCoral()) {
                        setState(States.IDLE);
                        canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(1.0));
                    }
                case OUTTAKE:
                    if (!hasAlgae() && !hasCoral()) {
                        setState(States.IDLE);
                        canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(0.0));
                    }
                case HAS_ALGAE:
                    if (!hasAlgae()) {
                        setState(States.IDLE);
                        canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(0.0));
                    }
                    pulseIntake();
                case IDLE:
                    if (hasCoral() || hasAlgae()) {
                        setState(hasAlgae() ? States.HAS_ALGAE : States.IDLE);
                        canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(1.0));
                    }
            }
        }
    }
}
