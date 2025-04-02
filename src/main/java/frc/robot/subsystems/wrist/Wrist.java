package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPoses;
import frc.robot.util.EquationUtil;

public class Wrist extends SubsystemBase {
    public enum State {
        MANUAL(Constants.WristConstants.LOWER_LIMIT.apply(0.0, 50.0)),
        STOW(Constants.WristConstants.STOW),
        CORAL_STATION(Constants.WristConstants.CORAL_STATION),
        CORAL_STATION_OBSTRUCTED(Constants.WristConstants.CORAL_STATION_OBSTRUCTED),
        GROUND_CORAL(Constants.WristConstants.GROUND_CORAL),
        GROUND_ALGAE(Constants.WristConstants.GROUND_ALGAE),
        L1_CORAL(Constants.WristConstants.L1_CORAL),
        L2_CORAL_AT_BRANCH(Constants.WristConstants.L2_CORAL_AT_BRANCH),
        L2_CORAL_ONE_CORAL_FROM_BRANCH(Constants.WristConstants.L2_CORAL_ONE_CORAL_FROM_BRANCH),
        L3_CORAL_AT_BRANCH(Constants.WristConstants.L3_CORAL_AT_BRANCH),
        L3_CORAL_ONE_CORAL_FROM_BRANCH(Constants.WristConstants.L3_CORAL_ONE_CORAL_FROM_BRANCH),
        L4_CORAL_AT_BRANCH(Constants.WristConstants.L4_CORAL_AT_BRANCH),
        L4_CORAL_ONE_CORAL_FROM_BRANCH(Constants.WristConstants.L4_CORAL_ONE_CORAL_FROM_BRANCH),
        LOW_REEF_ALGAE(Constants.WristConstants.LOW_REEF_ALGAE),
        HIGH_REEF_ALGAE(Constants.WristConstants.HIGH_REEF_ALGAE),
        PROCESSOR(Constants.WristConstants.PROCESSOR),
        NET(Constants.WristConstants.NET),
        CLIMB(Constants.WristConstants.CLIMB);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }


    private State currentState;

    private final TalonFX wrist;
    private final MotionMagicExpoVoltage request;
    private double target, error, lastManualPosition;

    private final WristTelemetry wristTelemetry = new WristTelemetry(this);

    public Wrist() {
        currentState = State.STOW;

        CANcoder encoder = new CANcoder(Constants.WristConstants.ENCODER_ID);
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.WristConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.WristConstants.ENCODER_ABSOLUTE_OFFSET)));

        wrist = new TalonFX(Constants.WristConstants.MOTOR_ID);
        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withRemoteCANcoder(encoder)
                        .withSensorToMechanismRatio(Constants.WristConstants.SENSOR_TO_DEGREE_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.WristConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.WristConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.WristConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKV(Constants.WristConstants.V)
                        .withKA(Constants.WristConstants.A)
                        .withKP(Constants.WristConstants.P)
                        .withKI(Constants.WristConstants.I)
                        .withKD(Constants.WristConstants.D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.WristConstants.EXPO_V)
                        .withMotionMagicExpo_kA(Constants.WristConstants.EXPO_A)));

        request = new MotionMagicExpoVoltage(0);

        target = State.STOW.position;
        error = 0.0;
        lastManualPosition = State.STOW.position;
    }

    public double getCurrent() {
        return wrist.getStatorCurrent().getValueAsDouble();
    }

    public State getState() {
        return currentState;
    }

    public double getTarget() {
        return target;
    }

    public double getPosition() {
        return wrist.getPosition().getValueAsDouble();
    }

    public double getError() {
        return error;
    }

    public boolean isAtState(State state) {
        return Math.abs(state.position - getPosition()) < Constants.WristConstants.AT_TARGET_TOLERANCE;
    }

    public boolean nearTarget() {
        return error < Constants.WristConstants.SAFE_TOLERANCE;
    }

    public boolean isAtTarget() {
        return error < Constants.WristConstants.AT_TARGET_TOLERANCE;
    }

    public void setTarget(double pivotPosition, double elevatorPosition) {
        this.target = MathUtil.clamp(
                getState() == State.MANUAL ? lastManualPosition : getState().position,
                Constants.WristConstants.LOWER_LIMIT.apply(elevatorPosition, pivotPosition),
                Constants.WristConstants.UPPER_LIMIT.apply(elevatorPosition)
        );
    }

    private void setState(State newState) {
        currentState = newState;
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

    public void setCoralStationObstructedState() {
        setState(State.CORAL_STATION_OBSTRUCTED);
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
            case ONE_CORAL_FROM_BRANCH -> setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setL3CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L3_CORAL_AT_BRANCH);
            case ONE_CORAL_FROM_BRANCH -> setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setL4CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L4_CORAL_AT_BRANCH);
            case ONE_CORAL_FROM_BRANCH -> setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setCoralObstructedState(boolean isObstructed) {
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

    public void holdTarget(double pivotPosition) {
        wrist.setControl(request.withPosition(target).withFeedForward(Constants.WristConstants.G.apply(getPosition(), pivotPosition)));
    }

    public void move(double axisValue, double pivotPosition, double elevatorPosition) {
        wrist.set(axisValue > 0
                ? axisValue * EquationUtil.expOutput(Constants.WristConstants.UPPER_LIMIT.apply(elevatorPosition) - getPosition(), 1, 5, 10)
                : axisValue * EquationUtil.expOutput(getPosition() - Constants.WristConstants.LOWER_LIMIT.apply(elevatorPosition, pivotPosition), 1, 5, 10));
    }

    @Override
    public void periodic() {
        wristTelemetry.publishValues();

        error = Math.abs(target - getPosition());
    }
}
