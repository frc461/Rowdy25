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
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.ExpUtil;
import frc.robot.util.FieldUtil;

public class Elevator extends SubsystemBase {
    public enum State {
        MANUAL(0.0),
        STOW(Constants.ElevatorConstants.LOWER_LIMIT),
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

    private State currentState;

    private final TalonFX elevator;
    private final Pivot pivot;
    private final DigitalInput lowerSwitch;
    private final MotionMagicExpoVoltage request;
    private double accuracy;

	private final ElevatorTelemetry elevatorTelemetry = new ElevatorTelemetry(this);

    public Elevator(Pivot pivot) { // TODO: FIGURE OUT HOW TO OBTAIN G CONSTANT WITHOUT PIVOT INSIDE SUBSYSTEM
        currentState = State.L2_CORAL; // TODO SHOP: TEST

        elevator = new TalonFX(Constants.ElevatorConstants.LEAD_ID);
        elevator.getConfigurator().apply(new TalonFXConfiguration() // TODO SHOP: TEST WITHOUT VOLTAGE CONSTRAINT
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Constants.ElevatorConstants.ROTOR_TO_INCH_RATIO))
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

        this.pivot = pivot;

        lowerSwitch = new DigitalInput(Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_ID);

        request = new MotionMagicExpoVoltage(0);

        elevator.setPosition(0.0); // TODO WAIT (LIMIT SWITCH IS AVAILABLE): REMOVE THIS
        accuracy = 1.0;
    }

	public State getState() {
		return currentState;
	}

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL ? getPosition() : getState().position;
    }

    public double getAlgaeHeight(Pose2d currentPose) {
        return FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(currentPose)) == FieldUtil.Reef.AlgaeLocation.UPPER
                ? Constants.ElevatorConstants.HIGH_REEF_ALGAE : Constants.ElevatorConstants.LOW_REEF_ALGAE;
    }

    public boolean lowerSwitchTriggered() {
        return false; // TODO WAIT (LIMIT SWITCH IS AVAILABLE): return !lowerSwitch.get();
    }

    public void setManualState() {
        setState(State.MANUAL);
    }

    public void toggleL2CoralState() {
        setState(getState() == State.L2_CORAL ? State.MANUAL : State.L2_CORAL);
    }

	public void setState(State state) {
		currentState = state;
	}

    public void checkLimitSwitch() {
        if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.ElevatorConstants.LOWER_LIMIT)) {
            elevator.setPosition(Constants.ElevatorConstants.LOWER_LIMIT);
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        elevator.setControl(request.withPosition(getTarget()).withFeedForward(Constants.ElevatorConstants.G.apply(pivot.getPosition())));
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
        }
    }

    @Override
    public void periodic() {
        elevatorTelemetry.publishValues();

        accuracy = getTarget() > getPosition() ? getPosition() / getTarget() : getTarget() / getPosition();
    }
}
