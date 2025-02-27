package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climb extends SubsystemBase {
    public enum State {
        UP(Constants.ClimbConstants.UP),
        DOWN(Constants.ClimbConstants.DOWN);

        private final int position;

        State(int position) {
            this.position = position;
        }
    }

    public enum LatchState {
        ON(Constants.ClimbConstants.LATCH_ON),
        OFF(Constants.ClimbConstants.LATCH_OFF);

        private final int pulseWidth;

        LatchState(int pulseWidth) {
            this.pulseWidth = pulseWidth;
        }
    }

    private State currentState;

    private LatchState currentLatchState;

	private final TalonFX climb;
    private final ServoChannel latch;

    private final ClimbTelemetry climbTelemetry = new ClimbTelemetry(this);
    
    public Climb() {
        currentState = State.DOWN;
        currentLatchState = LatchState.ON;

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

        latch = new ServoHub(Constants.ClimbConstants.SERVO_HUB_ID).getServoChannel(ServoChannel.ChannelId.kChannelId1);
        latch.setPowered(true);
        latch.setEnabled(true);
    }

    public void toggleState() {
        currentState = currentState == State.UP ? State.DOWN : State.UP;
    }

    public void setState(State state) {
        currentState = state;
    }

    public State getState() {
		return currentState;
	}

    public double getPosition() {
        return climb.getPosition().getValueAsDouble();
    }

    public void toggleRatchet() {
        currentLatchState = currentLatchState == LatchState.ON ? LatchState.OFF : Climb.LatchState.ON;
        latch.setPulseWidth(currentLatchState.pulseWidth);
    }

    public LatchState getCurrentLatchState() {
        return currentLatchState;
    }

    public void climbUp() {

    }

    public void climbDown() {

    }

    @Override
    public void periodic() {
        climbTelemetry.publishValues();

        latch.setPulseWidth(currentLatchState.pulseWidth);
    }
}
