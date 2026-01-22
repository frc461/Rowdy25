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

public class Intake extends SubsystemBase {
    public enum State {
        IDLE,
        HAS_ALGAE,
        INTAKE,
        INTAKE_SLOW,
        INTAKE_OUT,
        INTAKE_OVERRIDE,
        OUTTAKE,
        OUTTAKE_SLOW,
        OUTTAKE_L1
    }

    public enum StallIntent {
        CORAL_STUCK,
        HAS_ALGAE
    }

    private State currentState;

    private final TalonFX intake;
    private final Canandcolor canandcolor;
    private final DigitalInput beamBreak;

    private final IntakeTelemetry intakeTelemetry = new IntakeTelemetry(this);

    public StallIntent stallIntent = StallIntent.CORAL_STUCK;
    public Trigger hasAlgaeOrCoralStuck;
    private boolean maintainAlgaeCurrentOverride = false;
    private double proximityObjectDetectionThreshold = Constants.IntakeConstants.DEFAULT_PROXIMITY_OBJECT_DETECTION_THRESHOLD;
    public DoubleConsumer setProximityObjectDetectionThreshold = threshold -> proximityObjectDetectionThreshold = threshold;

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

    public double getCurrent() {
        return intake.getStatorCurrent().getValueAsDouble();
    }

    public State getState() {
        return currentState;
    }

    public double[] getColorReading() {
        return new double[] { canandcolor.getBlue(), canandcolor.getGreen(), canandcolor.getRed() };
    }

    public double getProximity() {
        return canandcolor.getProximity();
    }

    public boolean beamBreakBroken() {
        return !beamBreak.get();
    }

    public boolean coralEntered() {
        return getProximity() < proximityObjectDetectionThreshold;
    }

    public boolean barelyHasCoral() {
        return beamBreakBroken() || coralEntered();
    }

    public boolean hasCoral() {
        return beamBreakBroken() && coralEntered();
    }

    public boolean coralStuck() {
        return hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.CORAL_STUCK;
    }

    public boolean algaeStuck() {
        return  hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.HAS_ALGAE;
    }

    public boolean hasAlgae() {
        return maintainAlgaeCurrentOverride || hasAlgaeOrCoralStuck.getAsBoolean() && stallIntent == StallIntent.HAS_ALGAE;
    }

    public boolean atIdleState() {
        return currentState == State.IDLE;
    }

    public boolean atIntakeSlowState() {
        return currentState == State.INTAKE_SLOW;
    }

    public boolean atHasAlgaeState() {
        return currentState == State.HAS_ALGAE;
    }

    private void setState(State newState) {
        currentState = newState;
    }

    public void setIdleState() {
        if (hasAlgae()) {
            maintainAlgaeCurrentOverride = true;
            setState(State.HAS_ALGAE);
        } else {
            maintainAlgaeCurrentOverride = false;
            setState(State.IDLE);
        }
    }

    public void setAlgaeIntakeState() {
        stallIntent = StallIntent.HAS_ALGAE;
        maintainAlgaeCurrentOverride = false;
        setIntakeState(false);
    }

    public void setCoralIntakeState() {
        stallIntent = StallIntent.CORAL_STUCK;
        maintainAlgaeCurrentOverride = false;
        setIntakeState(false);
    }

    public void setIntakeState(boolean override) {
        if (override) {
            stallIntent = StallIntent.CORAL_STUCK;
            maintainAlgaeCurrentOverride = false;
            setState(State.INTAKE_OVERRIDE);
        } else {
            setState(State.INTAKE);
        }
    }

    public void setIntakeSlowState() {
        setState(State.INTAKE_SLOW);
    }

    public void setIntakeOutState() {
        setState(State.INTAKE_OUT);
    }

    public void setOuttakeState() {
        stallIntent = StallIntent.CORAL_STUCK;
        maintainAlgaeCurrentOverride = false;
        setState(State.OUTTAKE);
    }

    public void setOuttakeSlowState() {
        setState(State.OUTTAKE_SLOW);
    }

    public void setOuttakeL1State() {
        setState(State.OUTTAKE_L1);
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    @Override
    public void periodic() {
        intakeTelemetry.publishValues();

        Lights.setLights(hasCoral() || hasAlgae());
    }
}
