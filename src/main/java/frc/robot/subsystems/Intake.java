package frc.robot.subsystems;

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
    private CanandcolorSettings currentCanandcolorSettings;
    private States currentState;
    private boolean stateChanged;
    private int pulseCounter;

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
        pulseCounter = 0;
    }
 
    public boolean hasCoral() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kAqua); // TODO SHOP: Set the color to close to the color of coral
    }

    public boolean hasAlgae() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kWhiteSmoke); // TODO SHOP: Set the color to close to the color of algae
    }

    public Command intake() {
        return runOnce(() -> setState(States.INTAKE));
    }

    public Command outtake() {
        return runOnce(() -> setState(States.OUTTAKE));
    }

    private void setState(States state) {
        currentState = state;
        stateChanged = true;
    }

    private void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    private void pulseIntake(double speed) {
        if (pulseCounter > 10000) { 
            pulseCounter = 0; 
        } else if (pulseCounter++ < 5000) { 
            setIntakeSpeed(speed);
        }
    }

    @Override
    public void periodic() {
        Lights.setLights(hasCoral() || hasAlgae());

        switch (currentState) {
            case IDLE:
                setIntakeSpeed(0.0);
            case HAS_ALGAE:
                pulseIntake(0.1);
            case INTAKE:
                if (hasAlgae()) {
                    setState(States.HAS_ALGAE);
                    canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(1.0));
                } else if (hasCoral()) {
                    setState(States.IDLE);
                    canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(1.0));
                } else  {
                    setIntakeSpeed(0.75);
                }
            case OUTTAKE:
                if (!hasAlgae() && !hasCoral()) {
                    setState(States.IDLE);
                    canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(0.0));
                } else {
                    setIntakeSpeed(-0.5);
                }
        }

        // TODO: Add logic to get Canandcolor sensor data

    }
    
}
