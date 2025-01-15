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
import frc.robot.util.ExpUtil;

public class Wrist extends SubsystemBase {
    private final TalonFX wrist;
    private final PIDController controller;
    private final DigitalInput lowerLimitSwitch;
    private double target, error, accuracy;


    public Wrist() {
        wrist = new TalonFX(Constants.WristConstants.MOTOR_ID);

        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.WristConstants.WRIST_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.WristConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        controller = new PIDController(
            Constants.WristConstants.WRIST_P,
            Constants.WristConstants.WRIST_I,
            Constants.WristConstants.WRIST_D
        );

        lowerLimitSwitch = new DigitalInput(Constants.WristConstants.LOWER_LIMIT_SWITCH_ID);

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

    public double getError() {
        return error;
    }

    public boolean lowerSwitchTriggered() {
        return !lowerLimitSwitch.get();
    }

    public void checkLimitSwitch() {
       if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.WristConstants.LOWER_LIMIT)) {
           wrist.setPosition(Constants.WristConstants.LOWER_LIMIT);
       }
    }

    public void holdTarget(double height) {
        checkLimitSwitch();
        target = Math.max(Constants.WristConstants.LOWER_LIMIT, Math.min(Constants.WristConstants.UPPER_LIMIT, height));
        double output = controller.calculate(getPosition(), target);
        if (lowerSwitchTriggered()) {
            wrist.set(Math.max(0, output));
        } else if (getPosition() >= Constants.WristConstants.UPPER_LIMIT) { // TODO WE WANT UPPER SWITCHES!!
            wrist.set(Math.min(0, output));
        } else {
            wrist.set(output);
        }
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void moveAngle(double axisValue) {
        checkLimitSwitch();
        // TODO TUNE CURBING VALUE
        wrist.set(axisValue > 0
                ? axisValue * ExpUtil.output(Constants.WristConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * ExpUtil.output(getPosition() - Constants.WristConstants.LOWER_LIMIT, 1, 5, 10));
        target = getPosition();
    }
}