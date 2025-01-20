package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final MotionMagicExpoVoltage request;
    private final DigitalInput lowerSwitch;
    private double target, accuracy;

    public Elevator() {
        elevator = new TalonFX(Constants.ElevatorConstants.LEAD_ID);

        elevator.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ElevatorConstants.ELEVATOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKG(Constants.ElevatorConstants.ELEVATOR_G)
                        .withKS(Constants.ElevatorConstants.ELEVATOR_S)
                        .withKV(Constants.ElevatorConstants.ELEVATOR_V)
                        .withKA(Constants.ElevatorConstants.ELEVATOR_A)
                        .withKP(Constants.ElevatorConstants.ELEVATOR_P)
                        .withKI(Constants.ElevatorConstants.ELEVATOR_I)
                        .withKD(Constants.ElevatorConstants.ELEVATOR_D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.ElevatorConstants.ELEVATOR_V)
                        .withMotionMagicExpo_kA(Constants.ElevatorConstants.ELEVATOR_A)));

        try (TalonFX elevator2 = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID)) {
            elevator2.setControl(new Follower(Constants.ElevatorConstants.LEAD_ID, true));
        }

        request = new MotionMagicExpoVoltage(0);

        lowerSwitch = new DigitalInput(Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_ID);

        target = 0.0;
        accuracy = 1.0;
    }

   @Override
   public void periodic() {
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
   }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public boolean lowerSwitchTriggered() {
        return !lowerSwitch.get();
    }

    public void checkLimitSwitch() {
        if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.ElevatorConstants.LOWER_LIMIT)) {
            elevator.setPosition(Constants.ElevatorConstants.LOWER_LIMIT);
        }
    }

    public void holdTarget(double height) {
        checkLimitSwitch();
        target = Math.max(Constants.ElevatorConstants.LOWER_LIMIT, Math.min(Constants.ElevatorConstants.UPPER_LIMIT, height));
        elevator.setControl(request.withPosition(target));
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        // TODO TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            elevator.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.ElevatorConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.ElevatorConstants.LOWER_LIMIT, 1, 5, 10));
            target = getPosition();
        }
    }
}