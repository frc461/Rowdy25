package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;

public class Wrist extends SubsystemBase {
    private final TalonFX wrist;
    private final MotionMagicExpoVoltage request;
    private final DigitalInput lowerLimitSwitch; // TODO: ABSOLUTE ENCODERS, LIMIT SWITCHES NOT NEEDED
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
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKS(Constants.WristConstants.WRIST_S) // TODO: NEED G??????
                        .withKV(Constants.WristConstants.WRIST_V)
                        .withKA(Constants.WristConstants.WRIST_A)
                        .withKP(Constants.WristConstants.WRIST_P)
                        .withKI(Constants.WristConstants.WRIST_I)
                        .withKD(Constants.WristConstants.WRIST_D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.WristConstants.WRIST_V)
                        .withMotionMagicExpo_kA(Constants.WristConstants.WRIST_A)));

        lowerLimitSwitch = new DigitalInput(Constants.WristConstants.LOWER_LIMIT_SWITCH_ID);

        request = new MotionMagicExpoVoltage(0);

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
        wrist.setControl(request.withPosition(target));
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void moveWrist(double axisValue) {
        checkLimitSwitch();
        // TODO TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            wrist.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.WristConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.WristConstants.LOWER_LIMIT, 1, 5, 10));
            target = getPosition();
        }
    }
}