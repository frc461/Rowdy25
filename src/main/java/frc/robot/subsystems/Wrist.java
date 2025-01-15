package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Wrist extends SubsystemBase {
    private final TalonFX wrist;
    private final PIDController wristPIDController;
    private final DigitalInput lowerLimitSwitch;
    private double target, error, accuracy;


    public Wrist() {
        wrist = new TalonFX(Constants.WristConstants.WRIST_ID);

        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.WristConstants.WRIST_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.WristConstants.WRIST_CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        wristPIDController = new PIDController(
            Constants.WristConstants.WRIST_P,
            Constants.WristConstants.WRIST_I,
            Constants.WristConstants.WRIST_D
        );
        lowerLimitSwitch = new DigitalInput(Constants.WristConstants.WRIST_LOWER_LIMIT_SWITCH);

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    @Override
    public void periodic() {
        error = Math.abs(target - getPosition());
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
    }

    public double getPosition() { 
        return wrist.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public double wristVelocity() {
        return wrist.getVelocity().getValueAsDouble();
    }

    public double getError() {
        return error;
    }

    public boolean lowerSwitchTriggered() {
        return !lowerLimitSwitch.get();
    }

    public void checkLimitSwitches() {
       if (lowerSwitchTriggered()) {
           wrist.setPosition(Constants.WristConstants.WRIST_LOWER_LIMIT);
       }


    }

    public void holdTarget() {
        checkLimitSwitches();
        wrist.set(lowerSwitchTriggered() ? 0: wristPIDController.calculate(getPosition(), target));
    }

    public void moveAngle(double axisValue) {
        checkLimitSwitches();

        if (axisValue < 0 && lowerSwitchTriggered()) {
            target = Constants.WristConstants.WRIST_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && getPosition() > Constants.WristConstants.WRIST_UPPER_LIMIT) {
            target = Constants.WristConstants.WRIST_UPPER_LIMIT;
            holdTarget();
        } else {
            wrist.set(axisValue);
            target = wrist.getPosition().getValueAsDouble();
        }
    }

    public void setEncoderVal(double encoderVal) {
        checkLimitSwitches();
        encoderVal = Math.max(
                Constants.WristConstants.WRIST_LOWER_LIMIT,
                Math.min(encoderVal, Constants.WristConstants.WRIST_UPPER_LIMIT)
        );
        target = encoderVal;
        holdTarget();
    }
}