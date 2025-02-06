package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
        encoder = new CANcoder(Constants.WristConstants.ENCODER_ID); //TODO SHOP: CHECK IF THIS EXISTS
        encoder.getConfigurator().apply(new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(Constants.WristConstants.ENCODER_INVERT)
                        .withMagnetOffset(Constants.WristConstants.ENCODER_ABSOLUTE_OFFSET))); // TODO SHOP: CHECK AND POTENTIALLY ADD MORE CONFIGS

        wrist = new TalonFX(Constants.WristConstants.MOTOR_ID);
        wrist.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(Constants.WristConstants.PEAK_VOLTAGE))
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
                        .withKG(Constants.WristConstants.G) // TODO SHOP: NEED S??????
                        .withKV(Constants.WristConstants.V)
                        .withKA(Constants.WristConstants.A)
                        .withKP(Constants.WristConstants.P)
                        .withKI(Constants.WristConstants.I)
                        .withKD(Constants.WristConstants.D))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(0)
                        .withMotionMagicExpo_kV(Constants.WristConstants.V)
                        .withMotionMagicExpo_kA(Constants.WristConstants.A)));

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
