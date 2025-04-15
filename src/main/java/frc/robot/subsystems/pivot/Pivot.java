package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotPoses;
import frc.robot.util.EquationUtil;
import frc.robot.util.GravityGainsCalculator;
import frc.robot.subsystems.Lights;

public class Pivot extends SubsystemBase {
    public enum MotionMagicProfile {
        NORMAL(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0)
                .withMotionMagicExpo_kV(Constants.PivotConstants.EXPO_V)
                .withMotionMagicExpo_kA(Constants.PivotConstants.EXPO_A)),
        SLOW(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0)
                .withMotionMagicExpo_kV(Constants.PivotConstants.EXPO_V_SLOW)
                .withMotionMagicExpo_kA(Constants.PivotConstants.EXPO_A));

        final MotionMagicConfigs config;
        MotionMagicProfile(MotionMagicConfigs config) {
            this.config = config;
        }
    }

    public enum State {
        MANUAL(Constants.PivotConstants.LOWER_LIMIT),
        STOW(Constants.PivotConstants.STOW),
        L2_L3_L4_STOW(Constants.PivotConstants.L2_L3_L4_STOW),
        PERPENDICULAR(90.0),
        CORAL_STATION(Constants.PivotConstants.CORAL_STATION),
        CORAL_STATION_OBSTRUCTED(Constants.PivotConstants.CORAL_STATION_OBSTRUCTED),
        GROUND_CORAL(Constants.PivotConstants.GROUND_CORAL),
        GROUND_ALGAE(Constants.PivotConstants.GROUND_ALGAE),
        L1_CORAL(Constants.PivotConstants.L1_CORAL),
        L2_CORAL_AT_BRANCH(Constants.PivotConstants.L2_CORAL_AT_BRANCH),
        L2_CORAL_ONE_CORAL_FROM_BRANCH(Constants.PivotConstants.L2_CORAL_ONE_CORAL_FROM_BRANCH),
        L3_CORAL_AT_BRANCH(Constants.PivotConstants.L3_CORAL_AT_BRANCH),
        L3_CORAL_ONE_CORAL_FROM_BRANCH(Constants.PivotConstants.L3_CORAL_ONE_CORAL_FROM_BRANCH),
        L4_CORAL_AT_BRANCH(Constants.PivotConstants.L4_CORAL_AT_BRANCH),
        L4_CORAL_ONE_CORAL_FROM_BRANCH(Constants.PivotConstants.L4_CORAL_ONE_CORAL_FROM_BRANCH),
        LOW_REEF_ALGAE(Constants.PivotConstants.LOW_REEF_ALGAE),
        HIGH_REEF_ALGAE(Constants.PivotConstants.HIGH_REEF_ALGAE),
        PROCESSOR(Constants.PivotConstants.PROCESSOR),
        NET(Constants.PivotConstants.NET),
        PREPARE_CLIMB(Constants.PivotConstants.PREPARE_CLIMB),
        CLIMB(Constants.PivotConstants.CLIMB);

        private final double position;

        State(double position) {
            this.position = position;
        }
    }

    public enum RatchetState {
        ON(Constants.PivotConstants.DOWN_RATCHET_ON), // Pivot can move
        OFF(Constants.PivotConstants.DOWN_RATCHET_OFF); // Pivot cannot move

        private final int pulseWidth;

        RatchetState(int pulseWidth) {
            this.pulseWidth = pulseWidth;
        }
    }

    private State currentState;
    private MotionMagicProfile currentMotionMagicProfile;

    private final TalonFX pivot, intake;
    private final ServoChannel upRatchet, downRatchet;
    private final MotionMagicExpoVoltage request;

    private final GravityGainsCalculator gravityGainsCalculator = new GravityGainsCalculator(
            Constants.PivotConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_POSITION,
            Constants.WristConstants.AXIS_TO_ZERO_COM,
            Constants.ElevatorConstants.ZERO_UPRIGHT_COM,
            Constants.ElevatorConstants.COM_TO_STAGE_2_RATIO,
            Constants.ElevatorConstants.STAGE_3_LIMIT,
            Constants.ElevatorConstants.COM_TO_STAGE_3_RATIO,
            Constants.ElevatorConstants.MASS_LBS,
            Constants.WristConstants.MASS_LBS,
            Constants.PivotConstants.G
    );

    private double error, currentG, lastManualPosition;
    private boolean cageIntakeOverride, activateUpLatch;

    private final PivotTelemetry pivotTelemetry = new PivotTelemetry(this);

    public Pivot() {
        currentState = State.STOW;
        currentMotionMagicProfile = MotionMagicProfile.NORMAL;

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
                        .withInverted(Constants.PivotConstants.MOTOR_INVERT)
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
                .withMotionMagic(currentMotionMagicProfile.config));

        try (TalonFX pivot2 = new TalonFX(Constants.PivotConstants.FOLLOWER_ID)) {
            pivot2.setControl(new Follower(Constants.PivotConstants.LEAD_ID, true));
        }

        intake = new TalonFX(Constants.PivotConstants.INTAKE_ID);
        intake.getConfigurator().apply(new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs())
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.PivotConstants.INTAKE_MOTOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.PivotConstants.CURRENT_LIMIT))
        );

        upRatchet = Constants.SERVO_HUB.getServoChannel(Constants.PivotConstants.UP_RATCHET_CHANNEL);
        upRatchet.setEnabled(true);
        upRatchet.setPowered(true);

        downRatchet = Constants.SERVO_HUB.getServoChannel(Constants.PivotConstants.DOWN_RATCHET_CHANNEL);
        downRatchet.setEnabled(true);
        downRatchet.setPowered(true);

        request = new MotionMagicExpoVoltage(getTarget());

        error = 0.0;
        currentG = Constants.PivotConstants.G;
        lastManualPosition = State.STOW.position;
    }

    public double getCurrent() {
        return pivot.getStatorCurrent().getValueAsDouble();
    }

    public State getState() {
        return currentState;
    }

    public State getL2State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L2_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L2_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    public State getL3State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L3_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L3_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    public State getL4State(RobotPoses.Reef.RobotScoringSetting mode) {
        return switch (mode) {
            case L1, L2, AT_BRANCH -> State.L4_CORAL_AT_BRANCH;
            case ONE_CORAL_FROM_BRANCH -> State.L4_CORAL_ONE_CORAL_FROM_BRANCH;
        };
    }

    public State getCoralScoringObstructedState(boolean isObstructed) {
        return switch (currentState) {
            case L2_CORAL_AT_BRANCH -> isObstructed ? State.L2_CORAL_ONE_CORAL_FROM_BRANCH : State.L2_CORAL_AT_BRANCH;
            case L3_CORAL_AT_BRANCH -> isObstructed ? State.L3_CORAL_ONE_CORAL_FROM_BRANCH : State.L3_CORAL_AT_BRANCH;
            case L4_CORAL_AT_BRANCH -> isObstructed ? State.L4_CORAL_ONE_CORAL_FROM_BRANCH : State.L4_CORAL_AT_BRANCH;
            default -> currentState;
        };
    }

    public RatchetState getUpRatchetState() {
        return getState() == State.CLIMB ? RatchetState.OFF : RatchetState.ON;
    }

    public RatchetState getDownRatchetState() {
        return RatchetState.ON;
    }

    public double getPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return getState() == State.MANUAL ? lastManualPosition : getState().position;
    }

    public int getUpRatchetPulseWidth() {
        return activateUpLatch ? RatchetState.OFF.pulseWidth : RatchetState.ON.pulseWidth;
    }

    public int getDownRatchetPulseWidth() {
        return RatchetState.ON.pulseWidth;
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

    public double getUpRatchetStateValue() {
        return upRatchet.getPulseWidth();
    }

    public double getDownRatchetStateValue() {
        return downRatchet.getPulseWidth();
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

    public boolean goingThroughStow(State state) {
        return (state.position - State.STOW.position) * (getPosition() - State.STOW.position) < 0;
    }

    public void setNormalMotionMagicProfile() {
        currentMotionMagicProfile = MotionMagicProfile.NORMAL;
        pivot.getConfigurator().refresh(currentMotionMagicProfile.config);
    }

    public void setSlowMotionMagicProfile() {
        currentMotionMagicProfile = MotionMagicProfile.SLOW;
        pivot.getConfigurator().refresh(currentMotionMagicProfile.config);
    }

    private void setState(State newState) {
        activateUpLatch = newState == State.MANUAL ? activateUpLatch : newState == State.CLIMB;
        currentState = newState;
    }

    public void setManualState() {
        setState(State.MANUAL);
        lastManualPosition = getPosition();
    }

    public void setStowState() {
        setState(State.STOW);
    }

    public void setL2L3L4StowState() {
        setState(State.L2_L3_L4_STOW);
    }

    public void setPerpendicularState() {
        setState(State.PERPENDICULAR);
    }

    public void setCoralStationState() {
        setState(State.CORAL_STATION);
    }

    public void setCoralStationObstructedState() {
        setState(State.CORAL_STATION_OBSTRUCTED);
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

    public void setL2CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L2_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setL3CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L3_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setL4CoralState(RobotPoses.Reef.RobotScoringSetting mode) {
        switch (mode) {
            case AT_BRANCH -> setState(State.L4_CORAL_AT_BRANCH);
            case L2, ONE_CORAL_FROM_BRANCH -> setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
        }
    }

    public void setCoralScoringObstructedState(boolean isObstructed) {
        switch (currentState) {
            case L2_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L2_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
            case L3_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L3_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
            case L4_CORAL_AT_BRANCH:
                if (isObstructed) {
                    setState(State.L4_CORAL_ONE_CORAL_FROM_BRANCH);
                }
                break;
        }
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

    public void setPrepareClimbState() {
        setState(State.PREPARE_CLIMB);
    }

    public void setClimbState() {
        setSlowMotionMagicProfile();
        setState(State.CLIMB);
    }

    public void activateCageIntake() {
        cageIntakeOverride = true;
    }

    public void stopCageIntake() {
        cageIntakeOverride = false;
    }

    public void holdTarget(double elevatorPosition, double wristPosition) {
        currentG = gravityGainsCalculator.calculateGFromPositions(getPosition(), wristPosition, elevatorPosition);
        pivot.setControl(request.withPosition(getTarget()).withFeedForward(currentG));
    }

    public void move(double axisValue) {
        pivot.set(axisValue > 0
                ? axisValue * EquationUtil.expOutput(Constants.PivotConstants.UPPER_LIMIT - getPosition(), 1, 5, 10)
                : axisValue * EquationUtil.expOutput(getPosition() - Constants.PivotConstants.LOWER_LIMIT, 1, 5, 10));
    }

    @Override
    public void periodic() {
        pivotTelemetry.publishValues();

        error = Math.abs(getTarget() - getPosition());

        Lights.setLights((validStartPosition()) && DriverStation.isDisabled());

        upRatchet.setPulseWidth(getUpRatchetPulseWidth());
        downRatchet.setPulseWidth(getDownRatchetPulseWidth());

        if (currentState == State.PREPARE_CLIMB || cageIntakeOverride) {
            intake.set(0.6);
        } else {
            intake.set(0.0);
        }
    }
}
