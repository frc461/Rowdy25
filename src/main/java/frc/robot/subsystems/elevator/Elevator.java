package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotIdentity;
import frc.robot.constants.RobotPoses;
import frc.robot.util.EquationUtil;

public class Elevator extends SubsystemBase {
    public enum State {
        MANUAL(Constants.ElevatorConstants.LOWER_LIMIT),
        STOW(Constants.ElevatorConstants.STOW),
        CORAL_STATION(Constants.ElevatorConstants.CORAL_STATION),
        GROUND_ALGAE(Constants.ElevatorConstants.GROUND_ALGAE),
        GROUND_CORAL(Constants.ElevatorConstants.GROUND_CORAL),
        L1_CORAL(Constants.ElevatorConstants.L1_CORAL),
        L2_CORAL_AT_BRANCH(Constants.ElevatorConstants.L2_CORAL_AT_BRANCH),
        L2_CORAL_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L2_CORAL_ONE_CORAL_FROM_BRANCH),
        L2_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L2_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH),
        L3_CORAL_AT_BRANCH(Constants.ElevatorConstants.L3_CORAL_AT_BRANCH),
        L3_CORAL_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L3_CORAL_ONE_CORAL_FROM_BRANCH),
        L3_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L3_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH),
        L4_CORAL_AT_BRANCH(Constants.ElevatorConstants.L4_CORAL_AT_BRANCH),
        L4_CORAL_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L4_CORAL_ONE_CORAL_FROM_BRANCH),
        L4_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH(Constants.ElevatorConstants.L4_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH),
        LOW_REEF_ALGAE(Constants.ElevatorConstants.LOW_REEF_ALGAE),
        HIGH_REEF_ALGAE(Constants.ElevatorConstants.HIGH_REEF_ALGAE),
        NET(Constants.ElevatorConstants.NET),
        PROCESSOR(Constants.ElevatorConstants.PROCESSOR),
        CLIMB(Constants.ElevatorConstants.CLIMB);

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
                        .withNeutralMode(Constants.ElevatorConstants.NEUTRAL_MODE))
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

        lowerSwitch = new DigitalInput(Constants.ElevatorConstants.LOWER_LIMIT_SWITCH_DIO_PORT);

        request = new MotionMagicExpoVoltage(0);

        elevator.setPosition(0.0);
        error = 0.0;
        lastManualPosition = State.STOW.position;
    }

    public double getCurrent() {
        return elevator.getStatorCurrent().getValueAsDouble();
    }

    public double getRotorVelocity() {
        return elevator.getRotorVelocity().getValueAsDouble();
    }

	public State getState() {
		return currentState;
	}

    public State getL2State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L2_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L2_CORAL_ONE_CORAL_FROM_BRANCH;
            case FACING_AWAY_ONE_CORAL_FROM_BRANCH -> State.L2_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH;
        };
    }

    public State getL3State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L3_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L3_CORAL_ONE_CORAL_FROM_BRANCH;
            case FACING_AWAY_ONE_CORAL_FROM_BRANCH -> State.L3_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH;
        };
    }

    public State getL4State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L4_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L4_CORAL_ONE_CORAL_FROM_BRANCH;
            case FACING_AWAY_ONE_CORAL_FROM_BRANCH -> State.L4_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH;
        };
    }

    public State getCoralScoringObstructedState(boolean isObstructed) {
        return switch (currentState) {
            case L2_CORAL_AT_BRANCH -> isObstructed ? State.L2_CORAL_ONE_CORAL_FROM_BRANCH : State.L2_CORAL_AT_BRANCH;
            case L3_CORAL_AT_BRANCH -> isObstructed ? State.L3_CORAL_ONE_CORAL_FROM_BRANCH : State.L3_CORAL_AT_BRANCH;
            case L4_CORAL_AT_BRANCH -> isObstructed ? State.L4_CORAL_ONE_CORAL_FROM_BRANCH : State.L4_CORAL_AT_BRANCH;
            default -> currentState;
        };
    }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL ? lastManualPosition : getState().position;
    }
 
    public boolean lowerSwitchTriggered() {
        return Constants.IDENTITY != RobotIdentity.ROWDY && !lowerSwitch.get();
    }

    public boolean isAtState(State state) {
        return Math.abs(state.position - getPosition()) < Constants.ElevatorConstants.AT_TARGET_TOLERANCE;
    }

    public boolean nearTarget() {
        return error < Constants.ElevatorConstants.SAFE_TOLERANCE;
    }

    public boolean isAtTarget() {
        return error < Constants.ElevatorConstants.AT_TARGET_TOLERANCE;
    }

    public boolean goingDown(State state) {
        return getPosition() >= state.position;
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

    public void setL2CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L2_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
            case FACING_AWAY_ONE_CORAL_FROM_BRANCH -> setState(State.L2_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setL3CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L3_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
            case FACING_AWAY_ONE_CORAL_FROM_BRANCH -> setState(State.L3_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setL4CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L4_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
            case FACING_AWAY_ONE_CORAL_FROM_BRANCH -> setState(State.L4_CORAL_FACING_AWAY_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setCoralScoringObstructedState(boolean isObstructed) {
        switch (currentState) {
            case L2_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
            case L3_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
            case L4_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
        }
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

    public void setClimbState() {
        setState(State.CLIMB);
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

    public void move(double axisValue) {
        checkLimitSwitch();
        elevator.set(axisValue > 0
                ? axisValue * EquationUtil.expOutput(Constants.ElevatorConstants.UPPER_LIMIT - getPosition(), 1, 0.5, 10)
                : axisValue * EquationUtil.expOutput(getPosition() - Constants.ElevatorConstants.LOWER_LIMIT, 1, 0.5, 10));
    }

    @Override
    public void periodic() {
        elevatorTelemetry.publishValues();

        error = Math.abs(getPosition() - getTarget());
    }
}
