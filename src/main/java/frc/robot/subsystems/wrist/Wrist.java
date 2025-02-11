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
        NET(Constants.WristConstants.NET);

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
        wrist.getConfigurator().apply(new TalonFXConfiguration() // TODO SHOP: TEST WITHOUT VOLTAGE CONSTRAINT
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
                        .withKV(Constants.WristConstants.V) // TODO SHOP: NEED S??????
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

    public boolean isAtTarget() {
        return error < Constants.ElevatorConstants.TOLERANCE;
    }

    public void setTarget(double pivotPosition) {
        this.target = Math.max(getState() == State.MANUAL ? lastManualPosition : getState().position, Constants.WristConstants.LOWER_LIMIT.apply(pivotPosition));
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

    public void toggleCoralStationState() { // TODO SHOP: REMOVE TOGGLING ONCE TESTING IS FINISHED
        setState(currentState == State.CORAL_STATION ? State.STOW : State.CORAL_STATION);
    }

    public void toggleGroundCoralState() {
        setState(currentState == State.GROUND_CORAL ? State.STOW : State.GROUND_CORAL);
    }

    public void toggleGroundAlgaeState() {
        setState(currentState == State.GROUND_ALGAE ? State.STOW : State.GROUND_ALGAE);
    }

    public void toggleL1CoralState() {
        setState(currentState == State.L1_CORAL ? State.STOW : State.L1_CORAL);
    }

    public void toggleL2CoralState() {
        setState(currentState == State.L2_CORAL ? State.STOW : State.L2_CORAL);
    }

    public void toggleL3CoralState() {
        setState(currentState == State.L3_CORAL ? State.STOW : State.L3_CORAL);
    }

    public void toggleL4CoralState() {
        setState(currentState == State.L4_CORAL ? State.STOW : State.L4_CORAL);
    }

    public void toggleLowReefAlgaeState() {
        setState(currentState == State.LOW_REEF_ALGAE ? State.STOW : State.LOW_REEF_ALGAE);
    }

    public void toggleHighReefAlgaeState() {
        setState(currentState == State.HIGH_REEF_ALGAE ? State.STOW : State.HIGH_REEF_ALGAE);
    }

    public void toggleProcessorState() {
        setState(currentState == State.PROCESSOR ? State.STOW : State.PROCESSOR);
    }

    public void toggleNetState() {
        setState(currentState == State.NET ? State.STOW : State.NET);
    }

    public void holdTarget(double pivotPosition) {
        wrist.setControl(request.withPosition(target).withFeedForward(Constants.WristConstants.G.apply(getPosition(), pivotPosition)));
    }

    public void moveWrist(double axisValue, double pivotPosition) {
        // TODO SHOP: TUNE CURBING VALUE
        wrist.set(axisValue > 0
                ? axisValue * ExpUtil.output(Constants.WristConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * ExpUtil.output(getPosition() - Constants.WristConstants.LOWER_LIMIT.apply(pivotPosition), 1, 5, 10));
    }

    @Override
    public void periodic() {
        wristTelemetry.publishValues();

        error = Math.abs(target - getPosition());
    }
}
