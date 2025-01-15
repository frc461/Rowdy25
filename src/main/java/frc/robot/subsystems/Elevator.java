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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final PIDController elevatorPIDController;
    private final Servo elevatorClamp;
    private final DigitalInput elevatorSwitch, servoSwitch;
    private double target, accuracy;
    private boolean clamped, movingAboveLimitSwitch;

    public Elevator() {
        elevator = new TalonFX(Constants.ElevatorConstants.ELEVATOR_ID);


        elevator.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ElevatorConstants.ELEVATOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        elevatorPIDController = new PIDController(
                Constants.ElevatorConstants.ELEVATOR_P,
                Constants.ElevatorConstants.ELEVATOR_I,
                Constants.ElevatorConstants.ELEVATOR_D
        );

        try (TalonFX elevator2 = new TalonFX(Constants.ElevatorConstants.ELEVATOR_FOLLOWER_ID)) {
            elevator2.setControl(new Follower(Constants.ElevatorConstants.ELEVATOR_ID, true));
        }

        elevatorSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_LIMIT_SWITCH);
        servoSwitch = new DigitalInput(Constants.ElevatorConstants.SERVO_LIMIT_SWITCH);

        elevatorClamp = new Servo(Constants.ElevatorConstants.ELEVATOR_SERVO_PORT);
        elevatorClamp.set(Constants.ElevatorConstants.ELEVATOR_SERVO_UNCLAMPED_POS);

        target = 0.0;
        accuracy = 1.0;
        clamped = false; // disables/enables clamp
        movingAboveLimitSwitch = false; // whether the elevator is trying to move above the limit switch
    }

   @Override
   public void periodic() {
        if (!clamped) {
            elevatorClamp.set(Constants.ElevatorConstants.ELEVATOR_SERVO_UNCLAMPED_POS);
        } else {
            elevatorClamp.set(Constants.ElevatorConstants.ELEVATOR_SERVO_CLAMPED_POS);
        }
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
   }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public double getClampPosition() {
        return elevatorClamp.getPosition();
    }

    public double elevatorVelocity() {
        return elevator.getVelocity().getValueAsDouble();
    }

    public boolean isClamped() {
        return clamped;
    }

    public boolean elevatorSwitchTriggered() {
        return !elevatorSwitch.get();
    }

    public boolean servoSwitchTriggered() {
        return !servoSwitch.get();
    }

    public boolean nearTarget() {
        return accuracy > Constants.ElevatorConstants.ELEVATOR_ACCURACY_REQUIREMENT;
    }

    public void checkLimitSwitch() {
        if (elevatorSwitchTriggered()) {
            elevator.setPosition(Constants.ElevatorConstants.ELEVATOR_LOWER_LIMIT);
            if (nearTarget()) {
                movingAboveLimitSwitch = false;
            }
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        elevator.set(elevatorSwitchTriggered() && !movingAboveLimitSwitch ? 0 : elevatorPIDController.calculate(getPosition(), target));
    }

    public void climb(boolean stop) {
        if (stop) {
            setClamp(true);
            target = 0;
        } else {
            elevator.set(-0.5);
        }
    }

    // boolean toggles for specific subsystems are meant to be in the subsystem class, not robot container
    public void toggleClamp() {
        setClamp(!clamped);
    }

    public void setClamp(boolean toggle) {
        clamped = toggle;
        elevatorClamp.set(clamped ?
                Constants.ElevatorConstants.ELEVATOR_SERVO_CLAMPED_POS :
                Constants.ElevatorConstants.ELEVATOR_SERVO_UNCLAMPED_POS
        );
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.ElevatorConstants.ELEVATOR_LOWER_LIMIT;
            elevator.set(0);
        } else if (axisValue > 0 && getPosition() >= Constants.ElevatorConstants.ELEVATOR_UPPER_LIMIT) {
            target = Constants.ElevatorConstants.ELEVATOR_UPPER_LIMIT;
            holdTarget();
        } else {
            elevator.set(axisValue);
            target = getPosition();
        }
    }

    public void setHeight(double height) {
        checkLimitSwitch();
        height = Math.max(Constants.ElevatorConstants.ELEVATOR_LOWER_LIMIT, height);
        height = Math.min(Constants.ElevatorConstants.ELEVATOR_UPPER_LIMIT, height);
        target = height;
        movingAboveLimitSwitch = true;
        holdTarget();
    }

    public void stopElevator() {
        elevator.set(0);
    }
}