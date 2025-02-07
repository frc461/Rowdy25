package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;
import frc.robot.util.Lights;

public class Pivot extends SubsystemBase {
    private final TalonFX pivot;
    private final MotionMagicExpoVoltage request;
    private final ServoHub servoHub;
    private final ServoChannel ratchet;
    private double target, error, accuracy;
    private boolean ratcheted;

    private final PivotTelemetry pivotTelemetry = new PivotTelemetry(this);

    public Pivot() {
        CANcoder encoder = new CANcoder(Constants.PivotConstants.ENCODER_ID); //TODO SHOP: CHECK IF THIS EXISTS
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.PivotConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.PivotConstants.ENCODER_ABSOLUTE_OFFSET))); // TODO SHOP: CHECK AND POTENTIALLY ADD MORE CONFIGS

        pivot = new TalonFX(Constants.PivotConstants.LEAD_ID);
        pivot.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(Constants.PivotConstants.PEAK_VOLTAGE))
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
                        .withKG(Constants.PivotConstants.G) // TODO SHOP: NEED S??????
                        .withKV(Constants.PivotConstants.V)
                        .withKA(Constants.PivotConstants.A)
                        .withKP(Constants.PivotConstants.P)
                        .withKI(Constants.PivotConstants.I)
                        .withKD(Constants.PivotConstants.D)
                        .withGravityType(GravityTypeValue.Arm_Cosine))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.PivotConstants.V)
                        .withMotionMagicExpo_kA(Constants.PivotConstants.A)));

        try (TalonFX pivot2 = new TalonFX(Constants.PivotConstants.FOLLOWER_ID)) {
            pivot2.setControl(new Follower(Constants.PivotConstants.LEAD_ID, true));
        }

        servoHub = new ServoHub(Constants.PivotConstants.SERVO_HUB_ID);

        ratchet = servoHub.getServoChannel(ServoChannel.ChannelId.kChannelId0);
        ratchet.setEnabled(true);
        ratchet.setPowered(true);

        request = new MotionMagicExpoVoltage(0);

        ratcheted = true;

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    public double getPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
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

    public void holdTarget(double height) {
        target = Math.max(Constants.PivotConstants.LOWER_LIMIT, Math.min(Constants.PivotConstants.UPPER_LIMIT, height));
        pivot.setControl(request.withPosition(target));
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void movePivot(double axisValue) {
        // TODO SHOP: TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            pivot.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.PivotConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.PivotConstants.LOWER_LIMIT, 1, 5, 10));
            target = getPosition();
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

        error = Math.abs(target - getPosition());
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
    }
}
