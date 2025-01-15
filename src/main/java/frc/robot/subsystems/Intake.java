package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput coralBeam;
    private final DigitalInput algaeBeam;

    //TODO: ADD LIGHTS?

    public Intake() {
        leftMotor = new TalonFX(Constants.IntakeConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.IntakeConstants.RIGHT_MOTOR_ID);

        leftMotor.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.LEFT_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        rightMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(Constants.IntakeConstants.RIGHT_INVERT)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
            .withAudio(new AudioConfigs().withBeepOnConfig(false)
                    .withBeepOnBoot(false)
                    .withAllowMusicDurDisable(true)));

        coralBeam = new DigitalInput(Constants.IntakeConstants.CORAL_BEAM_ID);
        algaeBeam = new DigitalInput(Constants.IntakeConstants.ALGAE_BEAM_ID);
    }

    public void setIntakeSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void setIntakeSpeed(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    public boolean hasCoral() {
        return !coralBeam.get();
    }

    public boolean hasAlgae() {
        return !algaeBeam.get();
    }

    public boolean hasGamePiece() {
        return hasCoral() || hasAlgae();
    }

    @Override
    public void periodic() {
    
    }
    
}
