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
import frc.robot.util.ExpUtil;
import frc.robot.util.FieldUtil;

public class Elevator extends SubsystemBase {
    public enum State {
        MANUAL(Constants.ElevatorConstants.LOWER_LIMIT),
        STOW(Constants.ElevatorConstants.STOW),
        CORAL_STATION(Constants.ElevatorConstants.CORAL_STATION),
        GROUND_ALGAE(Constants.ElevatorConstants.GROUND_ALGAE),
        GROUND_CORAL(Constants.ElevatorConstants.GROUND_CORAL),
        L1_CORAL(Constants.ElevatorConstants.L1_CORAL),
        L2_CORAL(Constants.ElevatorConstants.L2_CORAL),
        L3_CORAL(Constants.ElevatorConstants.L3_CORAL),
        L4_CORAL(Constants.ElevatorConstants.L4_CORAL),
        LOW_REEF_ALGAE(Constants.ElevatorConstants.LOW_REEF_ALGAE),
        HIGH_REEF_ALGAE(Constants.ElevatorConstants.HIGH_REEF_ALGAE),
        NET(Constants.ElevatorConstants.NET),
        PROCESSOR(Constants.ElevatorConstants.PROCESSOR);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    private State currentState;

    private final TalonFX elevator;
    private final DigitalInput lowerSwitch;
    private final MotionMagicExpoVoltage request;
    private double error;

	private final ElevatorTelemetry elevatorTelemetry = new ElevatorTelemetry(this);

    public Elevator() {
        currentState = State.STOW;

        elevator = new TalonFX(Constants.ElevatorConstants.LEAD_ID);
        elevator.getConfigurator().apply(new TalonFXConfiguration()
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
                        .withKV(Constants.ElevatorConstants.V)
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

        elevator.setPosition(0.0); // TODO WAIT (LIMIT SWITCH IS AVAILABLE): REMOVE THIS
        error = 0.0;
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

    public double getAlgaeHeight(Pose2d currentPose) { // TODO: CHANGE STATE BASED ON CLOSEST ALGAE HEIGHT
        return FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(currentPose)) == FieldUtil.Reef.AlgaeLocation.UPPER
                ? Constants.ElevatorConstants.HIGH_REEF_ALGAE : Constants.ElevatorConstants.LOW_REEF_ALGAE;
    }

    public boolean lowerSwitchTriggered() {
        return false; // TODO WAIT (LIMIT SWITCH IS AVAILABLE): return !lowerSwitch.get();
    }

	private void setState(State state) {
		currentState = state;
	}

    public void setManualState() {
        setState(State.MANUAL);
    }

    public void setStowState() {
        setState(State.STOW);
    }

    public void toggleCoralStationState() {
        setState(getState() == State.CORAL_STATION ? State.STOW : State.CORAL_STATION);
    }

    public void toggleGroundCoralState() {
        setState(getState() == State.GROUND_CORAL ? State.STOW : State.GROUND_CORAL);
    }

    public void toggleGroundAlgaeState() {
        setState(getState() == State.GROUND_ALGAE ? State.STOW : State.GROUND_ALGAE);
    }

    public void toggleL1CoralState() {
        setState(getState() == State.L1_CORAL ? State.STOW : State.L1_CORAL);
    }

    public void toggleL2CoralState() {
        setState(getState() == State.L2_CORAL ? State.MANUAL : State.L2_CORAL);
    }

    public void toggleL3CoralState() {
        setState(getState() == State.L3_CORAL ? State.STOW : State.L3_CORAL);
    }

    public void toggleL4CoralState() {
        setState(getState() == State.L4_CORAL ? State.STOW : State.L4_CORAL);
    }

    public void toggleLowReefAlgaeState() {
        setState(getState() == State.LOW_REEF_ALGAE ? State.STOW : State.LOW_REEF_ALGAE);
    }

    public void toggleHighReefAlgaeState() {
        setState(getState() == State.HIGH_REEF_ALGAE ? State.STOW : State.HIGH_REEF_ALGAE);
    }

    public void toggleProcessorState() {
        setState(getState() == State.PROCESSOR ? State.STOW : State.PROCESSOR);
    }

    public void toggleNetState() {
        setState(getState() == State.NET ? State.STOW : State.NET);
    }

    public void checkLimitSwitch() {
        if (lowerSwitchTriggered() || (!lowerSwitchTriggered() && getPosition() <= Constants.ElevatorConstants.LOWER_LIMIT)) {
            elevator.setPosition(Constants.ElevatorConstants.LOWER_LIMIT);
        }
    }

    public void holdTarget(double pivotPosition) {
        checkLimitSwitch();
        elevator.setControl(request.withPosition(getTarget()).withFeedForward(Constants.ElevatorConstants.G.apply(pivotPosition)));
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        // TODO SHOP: TUNE CURBING VALUE
        elevator.set(axisValue > 0
                ? axisValue * ExpUtil.output(Constants.ElevatorConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * ExpUtil.output(getPosition() - Constants.ElevatorConstants.LOWER_LIMIT, 1, 5, 10));
    }

    @Override
    public void periodic() {
        elevatorTelemetry.publishValues();

        error = Math.abs(getPosition() - getTarget());
    }
}
