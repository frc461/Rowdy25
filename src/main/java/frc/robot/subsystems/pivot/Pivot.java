package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;
import frc.robot.util.Lights;

public class Pivot extends SubsystemBase {
    public enum State {
        MANUAL(0.0),
        STOW(Constants.PivotConstants.STOW_POSITION),
        SCORE_CORAL(Constants.PivotConstants.SCORE_CORAL),
        SCORE_ALGAE(Constants.PivotConstants.SCORE_ALGAE),
        GROUND_CORAL(Constants.PivotConstants.GROUND_CORAL),
        GROUND_ALGAE(Constants.PivotConstants.GROUND_ALGAE),
        CORAL_STATION(Constants.PivotConstants.CORAL_STATION);

        private final double position;

        State(double position) {
            this.position = position;
        }
    };

    private final TalonFX pivot;
    private State currentState;
    private final MotionMagicExpoVoltage request;
    private final ServoChannel ratchet;
    private double error, accuracy;
    private boolean ratcheted;

    private final PivotTelemetry pivotTelemetry = new PivotTelemetry(this);

    // TODO: STATES & COMMAND & VOID STATE CHANGERS
    // We want states for all pickup and scoring locations
    // Use the constants we have already created in default constants as the positions the pivot should go to for now (they are all 0)`
    // We need a default command (in the commands folder) that can change states while also accepting joystick inputs for manual control
    // State changers should go back to stow position if a button is pressed twice (just like the intake)

    public Pivot() {
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
                        .withKG(Constants.PivotConstants.G)
                        .withKV(Constants.PivotConstants.V) // TODO SHOP: NEED S??????
                        .withKA(Constants.PivotConstants.A)
                        .withKP(Constants.PivotConstants.P)
                        .withKI(Constants.PivotConstants.I)
                        .withKD(Constants.PivotConstants.D)
                        .withGravityType(GravityTypeValue.Arm_Cosine))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.PivotConstants.EXPO_V)
                        .withMotionMagicExpo_kA(Constants.PivotConstants.EXPO_A)));

        try (TalonFX pivot2 = new TalonFX(Constants.PivotConstants.FOLLOWER_ID)) {
            pivot2.setControl(new Follower(Constants.PivotConstants.LEAD_ID, true));
        }

        currentState = State.STOW;

        ratchet = new ServoHub(Constants.PivotConstants.SERVO_HUB_ID).getServoChannel(ServoChannel.ChannelId.kChannelId0);
        ratchet.setEnabled(true);
        ratchet.setPowered(true);

        request = new MotionMagicExpoVoltage(0);

        ratcheted = true;

        error = 0.0;
        accuracy = 1.0;
    }

    public State getState() {
        return currentState;
    }

    public double getPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL ? getPosition() : getState().position;
    }

    public double getError() {
        return error;
    }

    public double getRatchetValue() {
        return ratchet.getPulseWidth();
    }

    public boolean isRatcheted() {
        return ratcheted;
    }

    public void setManualState() {
        setState(State.MANUAL);
    }

    public void toggleScoreCoralState() {
        setState(currentState == State.SCORE_CORAL ? State.STOW : State.SCORE_CORAL);
    }

    public void toggleScoreAlgaeState() {
        setState(currentState == State.SCORE_ALGAE ? State.STOW : State.SCORE_ALGAE);
    }

    private void setState(State currentState) {
        this.currentState = currentState;
    }

    public void toggleRatchet() {
        setRatchet(!ratcheted);
    }

    public void setRatchet(boolean toggle) {
        ratcheted = toggle;
        ratchet.setPulseWidth(ratcheted ?
                Constants.PivotConstants.RATCHET_ON :
                Constants.PivotConstants.RATCHET_OFF
        );
    }

    public void holdTarget() {
        pivot.setControl(request.withPosition(getTarget()));;
    }


    public void movePivot(double axisValue) {
        // TODO SHOP: TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            pivot.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.PivotConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.PivotConstants.LOWER_LIMIT, 1, 5, 10));
        }
    }

    @Override
    public void periodic() {
        pivotTelemetry.publishValues();

        Lights.setLights((Math.abs(getPosition() - Constants.PivotConstants.STOW_POSITION) <= Constants.PivotConstants.TOLERANCE) && DriverStation.isDisabled());

        if (ratcheted) {
            ratchet.setPulseWidth(Constants.PivotConstants.RATCHET_ON) ;
        } else {
            ratchet.setPulseWidth(Constants.PivotConstants.RATCHET_OFF);
        }

        error = Math.abs(getTarget() - getPosition());
        accuracy = getTarget() > getPosition() ? getPosition() / getTarget() : getTarget() / getPosition();
    }
}
