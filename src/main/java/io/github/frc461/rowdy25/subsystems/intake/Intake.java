package io.github.frc461.rowdy25.subsystems.intake;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;

import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorPeriod;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.frc461.rowdy25.subsystems.Lights;

import io.github.frc461.rowdy25.constants.Constants;

import java.util.function.DoubleConsumer;

/**
 * Intake subsystem for acquiring and scoring coral and algae game pieces.
 * <p>
 * Manages motor control, proximity-based object detection via Canandcolor sensor,
 * beam break sensing, and state machine transitions for intake/outtake operations.
 * Supports both coral and algae detection with configurable proximity thresholds.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 */
public class Intake extends SubsystemBase {
    /** Intake state enumeration defining operational modes. */
    public enum State {
        /** Idle state - no intake or outtake. */
        IDLE,
        /** Holding algae state. */
        HAS_ALGAE,
        /** Normal intake speed. */
        INTAKE,
        /** Slow intake speed for precision loading. */
        INTAKE_SLOW,
        /** Slight reverse to reposition game piece during intake. */
        INTAKE_OUT,
        /** Intake at full speed, overriding normal limits. */
        INTAKE_OVERRIDE,
        /** Normal outtake speed to eject game piece. */
        OUTTAKE,
        /** Slow outtake speed for gentle ejection. */
        OUTTAKE_SLOW,
        /** Outtake speed for L1 scoring. */
        OUTTAKE_L1
    }

    /** Indicates what type of game piece the intake is trying to detect via current spike. */
    public enum StallIntent {
        /** Detecting a coral jam via current spike. */
        CORAL_STUCK,
        /** Detecting algae held in intake. */
        HAS_ALGAE
    }

    /** The current operational state of the intake. */
    private State currentState;

    /** The TalonFX motor controller for the intake. */
    private final TalonFX intake;

    /** The Canandcolor sensor for proximity and color detection. */
    private final Canandcolor canandcolor;

    /** The beam break sensor for detecting game piece presence. */
    private final DigitalInput beamBreak;

    /** Telemetry publisher for intake state. */
    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    /** The current stall detection intent (coral stuck vs. has algae). */
    public StallIntent stallIntent = StallIntent.CORAL_STUCK;

    /** Trigger that activates when motor current exceeds 40A for 0.1 seconds (indicating a game piece is held). */
    public Trigger hasAlgaeOrCoralStuck;

    /** Whether algae is being held and the idle state should maintain algae. */
    private boolean maintainAlgaeCurrentOverride = false;

    /** The proximity threshold below which a coral is considered entered. */
    private double proximityObjectDetectionThreshold = Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD;

    /** Consumer to allow external setting of the proximity detection threshold. */
    public DoubleConsumer setProximityObjectDetectionThreshold = threshold -> proximityObjectDetectionThreshold = threshold;

    /**
     * Constructs the Intake subsystem.
     * <p>
     * Initializes the TalonFX motor, Canandcolor proximity/color sensor with
     * 5ms proximity and 25ms color integration periods, beam break sensor on DIO,
     * and a debounced current-spike trigger for game piece detection.
     */
    public Intake() {
        intake = new TalonFX(Constants.IntakeConstants.MOTOR_ID);

        intake.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.MOTOR_INVERT)
                        .withNeutralMode(Constants.IntakeConstants.NEUTRAL_MODE))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        CanandEventLoop.getInstance();
        canandcolor = new Canandcolor(Constants.IntakeConstants.SENSOR_ID);
        canandcolor.setSettings(
                canandcolor.getSettings()
                        .setAlignProximityFramesToIntegrationPeriod(true)
                        .setProximityIntegrationPeriod(ProximityPeriod.k5ms)
                        .setAlignColorFramesToIntegrationPeriod(true)
                        .setColorIntegrationPeriod(ColorPeriod.k25ms)
                        .setDigoutFramePeriod(0.02)
        );
        canandcolor.setLampLEDBrightness(0.0);
        beamBreak = new DigitalInput(Constants.IntakeConstants.BEAMBREAK_DIO_PORT);
        currentState = State.IDLE;

        hasAlgaeOrCoralStuck = new Trigger(() -> Math.abs(getCurrent()) > 40.0).debounce(0.1, Debouncer.DebounceType.kRising);
    }

    /**
     * Returns the stator current draw of the intake motor.
     *
     * @return The current in amps.
     */
    public double getCurrent() {
        return intake.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Returns the current intake state.
     *
     * @return The current state.
     */
    public State getState() {
        return currentState;
    }

    /**
     * Returns the RGB color reading from the Canandcolor sensor.
     *
     * @return A double array of [blue, green, red] values.
     */
    public double[] getColorReading() {
        return new double[] { canandcolor.getBlue(), canandcolor.getGreen(), canandcolor.getRed() };
    }

    /**
     * Returns the proximity reading from the Canandcolor sensor.
     * Lower values indicate objects closer to the sensor.
     *
     * @return The proximity value.
     */
    public double getProximity() {
        return canandcolor.getProximity();
    }

    /**
     * Checks if the beam break sensor is broken (game piece detected).
     *
     * @return True if the beam is broken (sensor returns false).
     */
    public boolean beamBreakBroken() {
        return !beamBreak.get();
    }

    /**
     * Checks if a coral has entered the intake based on proximity threshold.
     *
     * @return True if proximity is below the detection threshold.
     */
    public boolean coralEntered() {
        return getProximity() < proximityObjectDetectionThreshold;
    }

    /**
     * Checks if a game piece is barely present (beam break OR proximity detected).
     *
     * @return True if either beam break or proximity indicates a game piece.
     */
    public boolean barelyHasCoral() {
        return beamBreakBroken() || coralEntered();
    }

    /**
     * Checks if a coral is fully present (beam break AND proximity detected).
     *
     * @return True if both beam break and proximity confirm a coral.
     */
    public boolean hasCoral() {
        return beamBreakBroken() && coralEntered();
    }

    /**
     * Checks if a coral is stuck in the intake (current spike + stall intent).
     *
     * @return True if the intake is stalling while trying to intake coral.
     */
    public boolean coralStuck() {
        return hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.CORAL_STUCK;
    }

    /**
     * Checks if algae is stuck in the intake (current spike + stall intent).
     *
     * @return True if the intake is stalling while algae is held.
     */
    public boolean algaeStuck() {
        return  hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.HAS_ALGAE;
    }

    /**
     * Checks if the intake has algae (via current spike or manual override).
     *
     * @return True if algae is present.
     */
    public boolean hasAlgae() {
        return maintainAlgaeCurrentOverride || hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.HAS_ALGAE;
    }

    /**
     * Checks if the intake is in the IDLE state.
     *
     * @return True if idle.
     */
    public boolean atIdleState() {
        return currentState == State.IDLE;
    }

    /**
     * Checks if the intake is in the INTAKE_SLOW state.
     *
     * @return True if in slow intake.
     */
    public boolean atIntakeSlowState() {
        return currentState == State.INTAKE_SLOW;
    }

    /**
     * Checks if the intake is in the HAS_ALGAE state.
     *
     * @return True if holding algae.
     */
    public boolean atHasAlgaeState() {
        return currentState == State.HAS_ALGAE;
    }

    /**
     * Sets the current intake state.
     *
     * @param newState The new state to set.
     */
    private void setState(State newState) {
        currentState = newState;
    }

    /**
     * Sets the intake to idle. If algae is present, transitions to HAS_ALGAE
     * instead of IDLE to maintain the current hold.
     */
    public void setIdleState() {
        if (hasAlgae()) {
            maintainAlgaeCurrentOverride = true;
            setState(State.HAS_ALGAE);
        } else {
            maintainAlgaeCurrentOverride = false;
            setState(State.IDLE);
        }
    }

    /** Sets the intake to algae intake mode and begins intake. */
    public void setAlgaeIntakeState() {
        stallIntent = StallIntent.HAS_ALGAE;
        maintainAlgaeCurrentOverride = false;
        setIntakeState(false);
    }

    /** Sets the intake to coral intake mode and begins intake. */
    public void setCoralIntakeState() {
        stallIntent = StallIntent.CORAL_STUCK;
        maintainAlgaeCurrentOverride = false;
        setIntakeState(false);
    }

    /**
     * Sets the intake to the intake state. If override is true, uses full speed
     * to push through jams; otherwise uses normal intake speed.
     *
     * @param override Whether to use the override (full speed) intake.
     */
    public void setIntakeState(boolean override) {
        if (override) {
            stallIntent = StallIntent.CORAL_STUCK;
            maintainAlgaeCurrentOverride = false;
            setState(State.INTAKE_OVERRIDE);
        } else {
            setState(State.INTAKE);
        }
    }

    /** Sets the intake to the slow intake state. */
    public void setIntakeSlowState() {
        setState(State.INTAKE_SLOW);
    }

    /** Sets the intake to the intake-out state (slight reverse during intake). */
    public void setIntakeOutState() {
        setState(State.INTAKE_OUT);
    }

    /** Sets the intake to the outtake state and resets stall intent to coral. */
    public void setOuttakeState() {
        stallIntent = StallIntent.CORAL_STUCK;
        maintainAlgaeCurrentOverride = false;
        setState(State.OUTTAKE);
    }

    /** Sets the intake to the slow outtake state. */
    public void setOuttakeSlowState() {
        setState(State.OUTTAKE_SLOW);
    }

    /** Sets the intake to the L1 outtake state. */
    public void setOuttakeL1State() {
        setState(State.OUTTAKE_L1);
    }

    /**
     * Directly sets the intake motor speed.
     *
     * @param speed The motor output speed (-1.0 to 1.0).
     */
    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    /**
     * Periodically updates telemetry and sets the LED lights based on whether a game piece
     * (coral or algae) is currently held in the intake.
     */
    @Override
    public void periodic() {
        intakeTelemetry.publishValues();

        Lights.setLights(hasCoral() || hasAlgae());
    }
}