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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.IntakeCommand;
import frc.robot.util.Lights;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    public enum States {
        IDLE,
        HAS_ALGAE,
        INTAKE,
        OUTTAKE
    }

    private final TalonFX motor;
    private final Canandcolor canandcolor; // TODO SHOP: Use https://docs.reduxrobotics.com/alchemist/ to configure IDs
    private final CanandcolorSettings currentCanandcolorSettings;
    private final Timer pulseTimer = new Timer();
    private States currentState;

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
        currentCanandcolorSettings = canandcolor.getSettings().setLampLEDBrightness(0.0);
        currentState = States.IDLE;
        pulseTimer.start();
    }

    public States getCurrentState() {
        return currentState;
    }

    public double[] getColorReading() {
        return new double[] { canandcolor.getBlue(), canandcolor.getGreen(), canandcolor.getRed() };
    }
 
    public boolean hasCoral() {
        return getColorReading()[0] > 0.5 && getColorReading()[1] > 0.5 && getColorReading()[2] > 0.5; // TODO SHOP: Set the color to close to the color of coral
    }

    public boolean hasAlgae() {
        return canandcolor.getColor().toWpilibColor().equals(Color.kAqua); // TODO SHOP: Set the color to close to the color of algae
    }

    public Command intakeCommand() {
        return new IntakeCommand(this);
    }

    public void toggleIntakeState() {
        setState(currentState == States.INTAKE ? States.IDLE : States.INTAKE);
    }

    public void toggleOuttakeState() {
        setState(currentState == States.OUTTAKE ? States.IDLE : States.OUTTAKE);
    }

    public void setState(States state) {
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
            canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(1.0));
        } else {
            Lights.setLights(false);
            canandcolor.setSettings(currentCanandcolorSettings.setLampLEDBrightness(0.0));
        }
    }
}
