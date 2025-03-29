package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;

public class Climb extends SubsystemBase {
    public enum State {
        STOW(Constants.ClimbConstants.STOW),
        MANUAL(Constants.ClimbConstants.STOW),
        MANUAL_LATCH(Constants.ClimbConstants.STOW),
        PREPARE_CLIMB(Constants.ClimbConstants.PREPARE_CLIMB),
        CLIMB(Constants.ClimbConstants.CLIMB);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    public enum IntakeState {
        IDLE,
        INTAKE;
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
    private IntakeState currentIntakeState;

	private final TalonFX climb;
    private final TalonFX intake;
    private final ServoChannel ratchet;
    private final Trigger isIntakeStalling;

    private double error;

    private final ClimbTelemetry climbTelemetry = new ClimbTelemetry(this);
    
    public Climb() {
        currentState = State.STOW;

        climb = new TalonFX(Constants.ClimbConstants.ID);
        climb.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs())
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ClimbConstants.MOTOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ClimbConstants.CURRENT_LIMIT))
        );

        currentIntakeState = IntakeState.IDLE;
        intake = new TalonFX(Constants.ClimbConstants.INTAKE_ID);
        intake.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs())
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.ClimbConstants.INTAKE_MOTOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.ClimbConstants.INTAKE_CURRENT_LIMIT))
        );

        isIntakeStalling = new Trigger(() -> intake.getStatorCurrent().getValueAsDouble() > Constants.ClimbConstants.INTAKE_CURRENT_LIMIT).debounce(0.75); // TODO SHOP: TEST STALLING ESPECIALLY DEBOUNCE THRESHOLD

        ratchet = Constants.SERVO_HUB.getServoChannel(Constants.ClimbConstants.RATCHET_CHANNEL);
        ratchet.setPowered(true);
        ratchet.setEnabled(true);

        climb.setPosition(0.0);
        error = 0.0;
    }

    public double getCurrent() {
        return climb.getStatorCurrent().getValueAsDouble();
    }

    public State getState() {
		return currentState;
	}

    public IntakeState getIntakeState() {
        return currentIntakeState;
    }
 
    public double getPosition() {
        return climb.getRotorPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL || getState() == State.MANUAL_LATCH ? getPosition() : getState().position;
    }

    public int getRatchetPulseWidth() {
        return getState() == State.CLIMB || getState() == State.MANUAL_LATCH ? RatchetState.OFF.pulseWidth : RatchetState.ON.pulseWidth;
    }

    public void escalate() {
        currentState = (currentState == State.CLIMB || currentState == State.PREPARE_CLIMB) ? State.CLIMB : State.PREPARE_CLIMB;
    }

    public void move(double value) {
        if (value >= 0) {
            currentIntakeState = IntakeState.INTAKE;
        } else {
            currentIntakeState = IntakeState.IDLE; // TODO SHOP: REMOVE? MIGHT BE UNNECESSARY
        }

        currentState = State.MANUAL;
        climb.set(value);
    }

    public void stop(boolean latched) {
        currentState = latched ? State.MANUAL_LATCH : State.MANUAL;
        climb.set(0.0);
    }

    public void reset() {
        currentState = State.STOW;
    }

    @Override
    public void periodic() {
        climbTelemetry.publishValues();

        error = getPosition() - getTarget();

        switch (getState()) {
            case STOW:
                if (Math.abs(error) > 15.0) {
                    climb.set(error < 0 ? 0.6 : -0.2);
                } else {
                    climb.set(0.0);
                }
                break;
            case PREPARE_CLIMB:
                currentIntakeState = IntakeState.INTAKE;
                if (Math.abs(error) > 7.5) {
                    climb.set(error < 0 ? 0.6 : -0.2);
                } else {
                    climb.set(0.0);
                }
                break;
            case CLIMB:
                currentIntakeState = IntakeState.IDLE;
                if (error > 7.5) {
                    climb.set(-0.6);
                } else {
                    climb.set(0.0);
                }
                break;
        }

        switch (getIntakeState()) {
            case INTAKE:
                if (isIntakeStalling.getAsBoolean()) {
                    currentIntakeState = IntakeState.IDLE; // TODO SHOP: TEST INTAKE STALLING LOGIC
                } else {
                    intake.set(0.6);
                }
                break;
            case IDLE:
                intake.set(0.0);
                break;
        }

        ratchet.setPulseWidth(getRatchetPulseWidth());
    }
}
