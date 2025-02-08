package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.Intake.State;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.ExpUtil;
import frc.robot.util.FieldUtil;

public class Elevator extends SubsystemBase {
    public enum State {
        IDLE(0.0),
        CORAL_STATION(Constants.ElevatorConstants.CORAL_STATION),
        GROUND_ALGAE(Constants.ElevatorConstants.GROUND_ALGAE),
        GROUND_CORAL(Constants.ElevatorConstants.GROUND_CORAL),
        HIGH_REEF_ALGAE(Constants.ElevatorConstants.HIGH_REEF_ALGAE),
        L1_CORAL(Constants.ElevatorConstants.L1_CORAL),
        L2_CORAL(Constants.ElevatorConstants.L2_CORAL),
        L3_CORAL(Constants.ElevatorConstants.L3_CORAL),
        L4_CORAL(Constants.ElevatorConstants.L4_CORAL),
        LOW_REEF_ALGAE(Constants.ElevatorConstants.LOW_REEF_ALGAE),
        NET(Constants.ElevatorConstants.NET),
        PROCESSOR(Constants.ElevatorConstants.PROCESSOR);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    private final Pivot pivot;
    private final TalonFX elevator;
    private final MotionMagicExpoVoltage request;
    private final DigitalInput lowerSwitch;
    private double target, accuracy;
    private State currentState;

	private final ElevatorTelemetry elevatorTelemetry = new ElevatorTelemetry(this);

    // TODO: STATES & COMMAND & VOID STATE CHANGERS

    public Elevator(Pivot pivot) {
        this.pivot = pivot;

        elevator = new TalonFX(Constants.ElevatorConstants.LEAD_ID);

        elevator.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(Constants.ElevatorConstants.PEAK_VOLTAGE)) // TODO: DETERMINE VOLTAGE
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Constants.ElevatorConstants.ROTOR_TO_PULLEY_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ElevatorConstants.MOTOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ElevatorConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKV(Constants.ElevatorConstants.V) // TODO SHOP: NEED S??????
                        .withKA(Constants.ElevatorConstants.A)
                        .withKP(Constants.ElevatorConstants.P)
                        .withKI(Constants.ElevatorConstants.I)
                        .withKD(Constants.ElevatorConstants.D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.ElevatorConstants.EXPO_V)
                        .withMotionMagicExpo_kA(Constants.ElevatorConstants.EXPO_A)));

        try (TalonFX elevator2 = new TalonFX(Constants.ElevatorConstants.FOLLOWER_ID)) {
            elevator2.setControl(new Follower(Constants.ElevatorConstants.LEAD_ID, true));
        }

        lowerSwitch = new DigitalInput(Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_ID);

        request = new MotionMagicExpoVoltage(0);

        target = 0.0;
        accuracy = 1.0;
        currentState = State.IDLE;
    }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

	public State getState() {
		return currentState;
	}

	public void setState(State state) {
		currentState = state;
	}

    public double getAlgaeHeight(Pose2d currentPose) {
        return FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(currentPose)) == FieldUtil.Reef.AlgaeLocation.UPPER
                ? Constants.ElevatorConstants.HIGH_REEF_ALGAE : Constants.ElevatorConstants.LOW_REEF_ALGAE;
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
        elevator.setControl(request.withPosition(target).withFeedForward(Constants.ElevatorConstants.G.apply(pivot.getPosition())));
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        // TODO SHOP: TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            elevator.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.ElevatorConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.ElevatorConstants.LOWER_LIMIT, 1, 5, 10));
            target = getPosition();
        }
    }

    @Override
    public void periodic() {
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();

        elevatorTelemetry.publishValues();
    }
}
