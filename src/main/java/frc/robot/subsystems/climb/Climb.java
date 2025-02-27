package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climb extends SubsystemBase {
    public enum State {
        IDLE(Constants.ClimbConstants.IDLE),
        PREPARE_CLIMB(Constants.ClimbConstants.PREPARE_CLIMB),
        CLIMB(Constants.ClimbConstants.CLIMB);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    public enum RatchetState {
        ON(Constants.ClimbConstants.RATCHET_ON),
        OFF(Constants.ClimbConstants.RATCHET_OFF);

        private final int pulseWidth;

        RatchetState(int pulseWidth) {
            this.pulseWidth = pulseWidth;
        }
    }

    private State currentState;

	private final TalonFX climb;
    private final ServoChannel ratchet;
    private double error;

    private final ClimbTelemetry climbTelemetry = new ClimbTelemetry(this);
    
    public Climb() {
        currentState = State.PREPARE_CLIMB;

        climb = new TalonFX(Constants.ClimbConstants.ID);
        climb.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Constants.ClimbConstants.ROTOR_TO_INCH_RATIO))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ClimbConstants.MOTOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ClimbConstants.CURRENT_LIMIT))
        );

        ratchet = Constants.SERVO_HUB.getServoChannel(Constants.ClimbConstants.RATCHET_CHANNEL);
        ratchet.setPowered(true);
        ratchet.setEnabled(true);

        climb.setPosition(0.0);
        error = 0.0;
    }

    public State getState() {
		return currentState;
	}

    public double getPosition() {
        return climb.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState().position;
    }

    public int getRatchetPulseWidth() {
        return getState() == State.CLIMB ? RatchetState.OFF.pulseWidth : RatchetState.ON.pulseWidth;
    }

    public void escalateClimb() {
        currentState = (currentState == State.CLIMB || currentState == State.PREPARE_CLIMB) ? State.CLIMB : State.PREPARE_CLIMB;
    }

    public void reset() {
        currentState = State.IDLE;
    }

    @Override
    public void periodic() {
        climbTelemetry.publishValues();

        error = getPosition() - getTarget();

        switch (getState()) {
            case IDLE:
                break;
            case PREPARE_CLIMB, CLIMB:
                if (Math.abs(error) > 5.0) {
                    climb.set(error < 0 ? 0.1 : -0.1);
                } else {
                    climb.set(0.0);
                }
                break;
        }

        ratchet.setPulseWidth(getRatchetPulseWidth());
    }
}
