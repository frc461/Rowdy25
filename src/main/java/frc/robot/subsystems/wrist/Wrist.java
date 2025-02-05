package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ExpUtil;

public class Wrist extends SubsystemBase {
    private final TalonFX wrist;
    private final CANcoder encoder;
    private final MotionMagicExpoVoltage request;
    private double target, error, accuracy;

    private final WristTelemetry wristTelemetry = new WristTelemetry(this);

    public Wrist() {
        wrist = new TalonFX(Constants.WristConstants.MOTOR_ID);
        encoder = new CANcoder(Constants.WristConstants.ENCODER_ID); //TODO SHOP: CHECK IF THIS EXISTS
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withMagnetOffset(Constants.WristConstants.ENCODER_ZERO_OFFSET))); // TODO SHOP: CHECK AND POTENTIALLY ADD MORE CONFIGS
            
        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withFeedback(new FeedbackConfigs().withRemoteCANcoder(encoder)
                        .withSensorToMechanismRatio(Constants.WristConstants.SENSOR_TO_DEGREE_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.WristConstants.WRIST_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.WristConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true))
                .withSlot0(new Slot0Configs()
                        .withKG(Constants.WristConstants.WRIST_G) // TODO SHOP: NEED S??????
                        .withKV(Constants.WristConstants.WRIST_V)
                        .withKA(Constants.WristConstants.WRIST_A)
                        .withKP(Constants.WristConstants.WRIST_P)
                        .withKI(Constants.WristConstants.WRIST_I)
                        .withKD(Constants.WristConstants.WRIST_D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.WristConstants.WRIST_V)
                        .withMotionMagicExpo_kA(Constants.WristConstants.WRIST_A)));

        request = new MotionMagicExpoVoltage(0);

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    public double getPosition() { 
        return wrist.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public double getError() {
        return error;
    }

    public void holdTarget(double height) {
        target = Math.max(Constants.WristConstants.LOWER_LIMIT, Math.min(Constants.WristConstants.UPPER_LIMIT, height));
        wrist.setControl(request.withPosition(target));
    }

    public void holdTarget() {
        holdTarget(target);
    }

    public void moveWrist(double axisValue) {
        // TODO SHOP: TUNE CURBING VALUE
        if (axisValue == 0) {
            holdTarget();
        } else {
            wrist.set(axisValue > 0
                    ? axisValue * ExpUtil.output(Constants.WristConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                    : axisValue * ExpUtil.output(getPosition() - Constants.WristConstants.LOWER_LIMIT, 1, 5, 10));
            target = getPosition();
        }
    }

    @Override
    public void periodic() {
        wristTelemetry.publishValues();

        error = Math.abs(target - getPosition());
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
    }
}
