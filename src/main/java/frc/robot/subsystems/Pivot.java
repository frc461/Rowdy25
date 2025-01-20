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

public class Pivot extends SubsystemBase {
    private final TalonFX pivot;
    private final PIDController controller;
    private final DigitalInput lowerLimitSwitch;
    private final DigitalInput upperLimitSwitch;
    private double target, error, accuracy;


    public Pivot() {
        pivot = new TalonFX(Constants.PivotConstants.MOTOR_ID);

        pivot.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.PivotConstants.PIVOT_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        controller = new PIDController(
            Constants.PivotConstants.PIVOT_P,
            Constants.PivotConstants.PIVOT_I,
            Constants.PivotConstants.PIVOT_D
        );

        lowerLimitSwitch = new DigitalInput(Constants.PivotConstants.LOWER_LIMIT_SWITCH_ID);
        upperLimitSwitch = new DigitalInput(Constants.PivotConstants.UPPER_LIMIT_SWITCH_ID);

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
        return pivot.getPosition().getValueAsDouble();
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

    public boolean upperSwitchTriggered() {
        return !upperLimitSwitch.get();
    }

    public void checkLimitSwitch() {
       if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.PivotConstants.LOWER_LIMIT)) {
           pivot.setPosition(Constants.PivotConstants.LOWER_LIMIT);
       }
       else if (upperSwitchTriggered() || (!upperSwitchTriggered() && getPosition() >= Constants.WristConstants.UPPER_LIMIT)) {
        pivot.setPosition(Constants.WristConstants.UPPER_LIMIT);
        }
    }

    public void holdTarget(double height) {
        checkLimitSwitch();
        target = Math.max(Constants.PivotConstants.LOWER_LIMIT, Math.min(Constants.PivotConstants.UPPER_LIMIT, height));
        double output = controller.calculate(getPosition(), target);
        if (lowerSwitchTriggered()) {
            pivot.set(Math.max(0, output));
        } else if (getPosition() >= Constants.PivotConstants.UPPER_LIMIT) { // TODO WE WANT UPPER SWITCHES!!
            pivot.set(Math.min(0, output));
        } else {
            pivot.set(output);
        }
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