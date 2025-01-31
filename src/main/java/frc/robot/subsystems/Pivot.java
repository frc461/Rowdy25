package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;
import frc.robot.util.Lights;

public class Pivot extends SubsystemBase {
    private final TalonFX pivot;
    private final MotionMagicExpoVoltage request;
    private final DigitalInput lowerLimitSwitch; // TODO: ABSOLUTE ENCODERS, LIMIT SWITCHES NOT NEEDED
    private final Servo ratchet;
    private double target, error, accuracy;
    private boolean ratcheted;

    public Pivot() {
        pivot = new TalonFX(Constants.PivotConstants.LEAD_ID);

        pivot.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.PivotConstants.PIVOT_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKS(Constants.PivotConstants.PIVOT_S) // TODO: NEED G??????
                        .withKV(Constants.PivotConstants.PIVOT_V)
                        .withKA(Constants.PivotConstants.PIVOT_A)
                        .withKP(Constants.PivotConstants.PIVOT_P)
                        .withKI(Constants.PivotConstants.PIVOT_I)
                        .withKD(Constants.PivotConstants.PIVOT_D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.PivotConstants.PIVOT_V)
                        .withMotionMagicExpo_kA(Constants.PivotConstants.PIVOT_A)));

        try (TalonFX pivot2 = new TalonFX(Constants.PivotConstants.FOLLOWER_ID)) {
            pivot2.setControl(new Follower(Constants.PivotConstants.LEAD_ID, true));
        }

        lowerLimitSwitch = new DigitalInput(Constants.PivotConstants.LOWER_LIMIT_SWITCH_ID);
        ratchet = new Servo(Constants.PivotConstants.RATCHET_ID);
        ratchet.set(Constants.PivotConstants.RATCHET_ON);

        request = new MotionMagicExpoVoltage(0);

        ratcheted = true;

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    @Override
    public void periodic() {
        Lights.setLights((Math.abs(getPosition() - Constants.PivotConstants.STOW_POSITION) <= Constants.PivotConstants.TOLERANCE) && DriverStation.isDisabled());

        if (ratcheted) {
            ratchet.set(Constants.PivotConstants.RATCHET_ON) ;
        } else {
            ratchet.set(Constants.PivotConstants.RATCHET_OFF);
        }

        error = Math.abs(target - getPosition());
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
    }

    public double getPosition() { 
        return pivot.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public double getError() {
        return error;
    }

    public boolean isRatcheted() {
        return ratcheted;
    }

    public void toggleRatchet() {
        setRatchet(!ratcheted);
    }

    public void setRatchet(boolean toggle) {
        ratcheted = toggle;
        ratchet.set(ratcheted ?
                Constants.PivotConstants.RATCHET_ON :
                Constants.PivotConstants.RATCHET_OFF
        );
    }

    public boolean lowerSwitchTriggered() {
        return !lowerLimitSwitch.get();
    }

    public void checkLimitSwitch() {
       if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.PivotConstants.LOWER_LIMIT)) {
           pivot.setPosition(Constants.PivotConstants.LOWER_LIMIT);
       }
    }

    public void holdTarget(double height) {
        checkLimitSwitch();
        target = Math.max(Constants.PivotConstants.LOWER_LIMIT, Math.min(Constants.PivotConstants.UPPER_LIMIT, height));
        pivot.setControl(request.withPosition(target));
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void movePivot(double axisValue) {
        checkLimitSwitch();
        // TODO TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            pivot.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.PivotConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.PivotConstants.LOWER_LIMIT, 1, 5, 10));
            target = getPosition();
        }
    }
}
