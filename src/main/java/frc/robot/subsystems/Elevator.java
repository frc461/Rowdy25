package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final PIDController elevatorPIDController;
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
                        .withAllowMusicDurDisable(true)));

        elevatorPIDController = new PIDController(
                Constants.ElevatorConstants.ELEVATOR_P,
                Constants.ElevatorConstants.ELEVATOR_I,
                Constants.ElevatorConstants.ELEVATOR_D
        );

        try (TalonFX elevator2 = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID)) {
            elevator2.setControl(new Follower(Constants.ElevatorConstants.LEAD_ID, true));
        }

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
        double output = elevatorPIDController.calculate(getPosition(), target);
        if (lowerSwitchTriggered()) {
            elevator.set(Math.max(0, output));
        } else if (getPosition() >= Constants.ElevatorConstants.UPPER_LIMIT) {
            elevator.set(Math.min(0, output));
        } else {
            elevator.set(output);
        }
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        // TODO TUNE CURBING VALUE
        elevator.set(axisValue > 0
                ? axisValue * ExpUtil.output(Constants.ElevatorConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * ExpUtil.output(getPosition() - Constants.ElevatorConstants.LOWER_LIMIT, 1, 5, 10));
        target = getPosition();
    }

    public void stopElevator() {
        elevator.set(0);
    }
}