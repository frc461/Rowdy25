package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;

public class Wrist extends SubsystemBase {
    public enum State {
        MANUAL(Constants.WristConstants.LOWER_LIMIT.apply(50.0)),
        STOW(Constants.WristConstants.STOW),
        CORAL_STATION(Constants.WristConstants.CORAL_STATION),
        GROUND_CORAL(Constants.WristConstants.GROUND_CORAL),
        GROUND_ALGAE(Constants.WristConstants.GROUND_ALGAE),
        L1_CORAL(Constants.WristConstants.L1_CORAL),
        L2_CORAL(Constants.WristConstants.L2_CORAL),
        L3_CORAL(Constants.WristConstants.L3_CORAL),
        L4_CORAL(Constants.WristConstants.L4_CORAL),
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
                Constants.WristConstants.LOWER_LIMIT.apply(pivotPosition),
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

    public void setClimbState() {
        setState(State.CLIMB);
    }

    public void holdTarget(double pivotPosition) {
        wrist.setControl(request.withPosition(target).withFeedForward(Constants.WristConstants.G.apply(getPosition(), pivotPosition)));
    }

    public void moveWrist(double axisValue, double pivotPosition, double elevatorPosition) {
        wrist.set(axisValue > 0
                ? axisValue * ExpUtil.output(Constants.WristConstants.UPPER_LIMIT.apply(elevatorPosition) - getPosition(), 1, 5, 10)
                : axisValue * ExpUtil.output(getPosition() - Constants.WristConstants.LOWER_LIMIT.apply(pivotPosition), 1, 5, 10));
    }

    @Override
    public void periodic() {
        wristTelemetry.publishValues();

        error = Math.abs(target - getPosition());
    }
}
