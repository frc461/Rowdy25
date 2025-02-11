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
    private double error, lastManualPosition;

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
        lastManualPosition = State.STOW.position;
    }

	public State getState() {
		return currentState;
	}

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL ? lastManualPosition : getState().position;
    }

    public boolean getHighAlgae(Pose2d currentPose) { // TODO: CHANGE STATE BASED ON CLOSEST ALGAE HEIGHT
        return FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(currentPose)) == FieldUtil.Reef.AlgaeLocation.UPPER; 
    }

    public boolean lowerSwitchTriggered() {
        return false; // TODO WAIT (LIMIT SWITCH IS AVAILABLE): return !lowerSwitch.get();
    }

    public boolean isAtTarget() {
        return error < Constants.ElevatorConstants.TOLERANCE;
    }

	private void setState(State state) {
		currentState = state;
	}

    public void setManualState() {
        setState(State.MANUAL);
        lastManualPosition = getPosition();
    }

    public void setStowState() {
        setState(State.STOW);
    }

    public void setCoralStationState() {
        setState(State.CORAL_STATION);
    }

    public void setGroundCoralState() {
        setState(State.GROUND_CORAL);
    }

    public void setGroundAlgaeState() {
        setState(State.GROUND_ALGAE);
    }

    public void setL1CoralState() {
        setState(State.L1_CORAL);
    }

    public void setL2CoralState() {
        setState(State.L2_CORAL);
    }

    public void setL3CoralState() {
        setState(State.L3_CORAL);
    }

    public void setL4CoralState() {
        setState(State.L4_CORAL);
    }

    public void setLowReefAlgaeState() {
        setState(State.LOW_REEF_ALGAE);
    }

    public void setHighReefAlgaeState() {
        setState(State.HIGH_REEF_ALGAE);
    }

    public void setProcessorState() {
        setState(State.PROCESSOR);
    }

    public void setNetState() {
        setState(State.NET);
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
