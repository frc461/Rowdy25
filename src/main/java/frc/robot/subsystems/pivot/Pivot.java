package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;
import frc.robot.util.GravityGainsCalculator;
import frc.robot.subsystems.Lights;

public class Pivot extends SubsystemBase {
    public enum State {
        MANUAL(Constants.PivotConstants.LOWER_LIMIT),
        STOW(Constants.PivotConstants.STOW),
        CORAL_STATION(Constants.PivotConstants.CORAL_STATION),
        GROUND_CORAL(Constants.PivotConstants.GROUND_CORAL),
        GROUND_ALGAE(Constants.PivotConstants.GROUND_ALGAE),
        L1_CORAL(Constants.PivotConstants.L1_CORAL),
        L2_CORAL(Constants.PivotConstants.L2_CORAL),
        L3_CORAL(Constants.PivotConstants.L3_CORAL),
        L4_CORAL(Constants.PivotConstants.L4_CORAL),
        LOW_REEF_ALGAE(Constants.PivotConstants.LOW_REEF_ALGAE),
        HIGH_REEF_ALGAE(Constants.PivotConstants.HIGH_REEF_ALGAE),
        PROCESSOR(Constants.PivotConstants.PROCESSOR),
        NET(Constants.PivotConstants.NET),
        CLIMB(Constants.PivotConstants.CLIMB);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    public enum RatchetState {
        ON(Constants.PivotConstants.RATCHET_ON), // Pivot can move
        OFF(Constants.PivotConstants.RATCHET_OFF); // Pivot cannot move

        private final int pulseWidth;

        RatchetState(int pulseWidth) {
            this.pulseWidth = pulseWidth;
        }
    }

    private State currentState;
    private RatchetState currentRatchetState;

    private final TalonFX pivot;
    private final ServoChannel ratchet;
    private final MotionMagicExpoVoltage request;

    private final GravityGainsCalculator gravityGainsCalculator = new GravityGainsCalculator(
            Constants.PivotConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_TO_ZERO_COM,
            Constants.ElevatorConstants.ZERO_UPRIGHT_COM,
            Constants.ElevatorConstants.COM_TO_STAGE_2_RATIO,
            Constants.ElevatorConstants.STAGE_2_LIMIT,
            Constants.ElevatorConstants.COM_TO_STAGE_3_RATIO,
            Constants.ElevatorConstants.MASS_LBS,
            Constants.WristConstants.MASS_LBS,
            Constants.PivotConstants.G
    );

    private double error, currentG, lastManualPosition;

    private final PivotTelemetry pivotTelemetry = new PivotTelemetry(this);

    public Pivot() {
        currentState = State.STOW;
        currentRatchetState = RatchetState.ON;

        CANcoder encoder = new CANcoder(Constants.PivotConstants.ENCODER_ID);
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.PivotConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.PivotConstants.ENCODER_ABSOLUTE_OFFSET)));

        pivot = new TalonFX(Constants.PivotConstants.LEAD_ID);
        pivot.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withRemoteCANcoder(encoder)
                        .withSensorToMechanismRatio(Constants.PivotConstants.SENSOR_TO_DEGREE_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.PivotConstants.PIVOT_INVERT)
                        .withNeutralMode(Constants.PivotConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKV(Constants.PivotConstants.V)
                        .withKA(Constants.PivotConstants.A)
                        .withKP(Constants.PivotConstants.P)
                        .withKI(Constants.PivotConstants.I)
                        .withKD(Constants.PivotConstants.D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.PivotConstants.EXPO_V)
                        .withMotionMagicExpo_kA(Constants.PivotConstants.EXPO_A)));

        try (TalonFX pivot2 = new TalonFX(Constants.PivotConstants.FOLLOWER_ID)) {
            pivot2.setControl(new Follower(Constants.PivotConstants.LEAD_ID, true));
        }

        ratchet = Constants.SERVO_HUB.getServoChannel(Constants.PivotConstants.RATCHET_CHANNEL);
        ratchet.setEnabled(true);
        ratchet.setPowered(true);

        request = new MotionMagicExpoVoltage(getTarget());

        error = 0.0;
        currentG = Constants.PivotConstants.G;
        lastManualPosition = State.STOW.position;
    }

    public State getState() {
        return currentState;
    }

    public RatchetState getRatchetState() {
        return currentRatchetState;
    }

    public double getPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL ? lastManualPosition : getState().position;
    }

    public boolean validStartPosition() {
        return Math.abs(getPosition() - Constants.PivotConstants.STOW) <= Constants.PivotConstants.SAFE_TOLERANCE;
    }

    public double getError() {
        return error;
    }

    public double getCurrentGravityGains() {
        return currentG;
    }

    public double getRatchetStateValue() {
        return ratchet.getPulseWidth();
    }

    public boolean isAtState(State state) {
        return Math.abs(state.position - getPosition()) < Constants.PivotConstants.AT_TARGET_TOLERANCE;
    }

    public boolean nearTarget() {
        return error < Constants.PivotConstants.SAFE_TOLERANCE;
    }

    public boolean isAtTarget() {
        return error < Constants.PivotConstants.AT_TARGET_TOLERANCE;
    }

    public void toggleRatchet() {
        currentRatchetState = currentRatchetState == RatchetState.ON ? RatchetState.OFF : RatchetState.ON;
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

    public void holdTarget(double elevatorPosition, double wristPosition) {
        currentG = gravityGainsCalculator.calculateGFromPositions(getPosition(), wristPosition, elevatorPosition);
        pivot.setControl(request.withPosition(getTarget()).withFeedForward(currentG));
    }


    public void movePivot(double axisValue) {
        pivot.set(axisValue > 0
                ? axisValue * ExpUtil.output(Constants.PivotConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * ExpUtil.output(getPosition() - Constants.PivotConstants.LOWER_LIMIT, 1, 5, 10));
    }

    @Override
    public void periodic() {
        pivotTelemetry.publishValues();

        error = Math.abs(getTarget() - getPosition());

        Lights.setLights((validStartPosition()) && DriverStation.isDisabled());

        ratchet.setPulseWidth(currentRatchetState.pulseWidth);
    }
}
