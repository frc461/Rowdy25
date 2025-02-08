package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.ExpUtil;

public class Wrist extends SubsystemBase {
    public enum State {
        MANUAL(0.0),
        GROUND_CORAL(Constants.WristConstants.GROUND_CORAL),
        GROUND_ALGAE(Constants.WristConstants.GROUND_ALGAE),
        L1_CORAL(Constants.WristConstants.L1_CORAL),
        L2_L3_CORAL(Constants.WristConstants.L2_L3_CORAL),
        L4_CORAL(Constants.WristConstants.L4_CORAL),
        REEF_ALGAE(Constants.WristConstants.REEF_ALGAE),
        PROCESSOR(Constants.WristConstants.PROCESSOR),
        NET(Constants.WristConstants.NET),
        STOW(Constants.WristConstants.STOW_POSITION);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }


    private State state;

    private final Pivot pivot;
    private final TalonFX wrist;
    private final MotionMagicExpoVoltage request;
    private double error, accuracy;

    private final WristTelemetry wristTelemetry = new WristTelemetry(this);

    // TODO: STATES & COMMAND & VOID STATE CHANGERS

    public Wrist(Pivot pivot) {
        this.pivot = pivot;

        CANcoder encoder = new CANcoder(Constants.WristConstants.ENCODER_ID);
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.WristConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.WristConstants.ENCODER_ABSOLUTE_OFFSET))); // TODO SHOP: CHECK AND POTENTIALLY ADD MORE CONFIGS

        wrist = new TalonFX(Constants.WristConstants.MOTOR_ID);
        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(Constants.WristConstants.PEAK_VOLTAGE)) // TODO: DETERMINE VOLTAGE
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

        error = 0.0;
        accuracy = 1.0;

        state = State.STOW;
    }

    public double getPosition() { 
        return wrist.getPosition().getValueAsDouble();
    }


    public double getTarget() {
        return getState() == State.MANUAL ? getPosition() : getState().position;
    }

    public double getError() {
        return error;
    }

    public void holdTarget() {
        wrist.setControl(request.withPosition(getTarget()).withFeedForward(Constants.WristConstants.G.apply(getPosition(), pivot.getPosition())));
    }

    public void moveWrist(double axisValue) {
        // TODO SHOP: TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            wrist.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.WristConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.WristConstants.LOWER_LIMIT, 1, 5, 10));
        }
    }

    public void setManualState() {
        setState(State.MANUAL);
    }

    public void toggleGroundCoral() {
        setState(state == State.GROUND_CORAL ? State.STOW : State.GROUND_CORAL);
    }

    public void toggleGroundAlgae() {
        setState(state == State.GROUND_ALGAE ? State.STOW : State.GROUND_ALGAE);
    }

    private void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        wristTelemetry.publishValues();

        error = Math.abs(getTarget() - getPosition());
        accuracy = getTarget() > getPosition()
                    ? getPosition() / getTarget()
                    : getTarget() / getPosition();
    }
}
