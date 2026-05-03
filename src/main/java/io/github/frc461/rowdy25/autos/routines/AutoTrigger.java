package io.github.frc461.rowdy25.autos.routines;

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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Represents a conditional trigger within an autonomous routine, tied to an {@link AutoEventLooper}.
 * This class tracks the state of a specified command (active, finished, or interrupted) and provides
 * {@link Trigger} instances to observe these states during the autonomous phase.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class AutoTrigger {
    /** The specified name of the trigger. */
    public final String name;
    
    /** The autonomous event looper managing this trigger. */
    private final AutoEventLooper auto;
    
    /** The supplier for the command this trigger executes and tracks. */
    private final Supplier<Command> triggeredCommand;
    
    /** The cached, decorated command instance that incorporates state-tracking hooks. */
    private Command decoratedCommand = null;

    /** Indicates whether the command is currently running. */
    public boolean isActive = false;
    
    /** Indicates whether the command has completed execution naturally (without being interrupted). */
    public boolean isFinished = false;
    
    /** Indicates whether the command was interrupted during execution. */
    public boolean interrupted = false;

    /**
     * Constructs an {@link AutoTrigger} with the specified name, command supplier, and event looper.
     *
     * @param name The name of the trigger.
     * @param command A supplier providing the command to be executed.
     * @param auto The autonomous event looper associated with this trigger.
     */
    public AutoTrigger(String name, Supplier<Command> command, AutoEventLooper auto) {
        this.name = name;
        this.auto = auto;
        this.triggeredCommand = command;
    }

    /**
     * Creates and returns a new {@link AutoTrigger} instance with identical properties to this one.
     *
     * @return A duplicated {@link AutoTrigger}.
     */
    public AutoTrigger duplicate() {
        return new AutoTrigger(this.name, this.triggeredCommand, this.auto);
    }

    /**
     * Lazily decorates and returns the command to be executed. The decorated command includes
     * initialization and completion hooks to update the trigger's internal state (active, finished, interrupted).
     *
     * @return The decorated {@link Command} instance.
     */
    public Command cmd() {
        if (decoratedCommand == null) {
            decoratedCommand = triggeredCommand.get().finallyDo(
                    interrupted -> {
                        isActive = false;
                        isFinished = !interrupted;
                        this.interrupted = interrupted;
                    }
            ).beforeStarting(
                    () -> {
                        isActive = true;
                        isFinished = false;
                        interrupted = false;
                    }
            ).withName(name);
        }
        return decoratedCommand;
    }

    /**
     * Returns a {@link Trigger} that is true while this autonomous routine is being polled and this trigger is active.
     *
     * <p>Using a {@link Trigger#onFalse(Command)} will do nothing as when this is false the routine
     * is not being polled anymore.
     *
     * @return A {@link Trigger} that is true while this trigger is active during autonomous polling.
     */
    public Trigger active() {
        return auto.observe(() -> isActive && auto.isActive);
    }

    /**
     * Returns a {@link Trigger} that is true when this trigger is not active.
     *
     * @return A {@link Trigger} representing the negated active state.
     */
    public Trigger inactive() {
        return active().negate();
    }

    /**
     * Returns a {@link Trigger} that is true when this trigger is inactive and was interrupted.
     *
     * @return A {@link Trigger} observing the interrupted state.
     */
    public Trigger interrupt() {
        return inactive().and(auto.observe(() -> interrupted));
    }

    /**
     * Returns a {@link Trigger} that evaluates to true a specific number of polling cycles after this trigger finishes.
     *
     * @param cyclesToDelay The number of event loop cycles to delay before triggering.
     * @return A {@link Trigger} representing the delayed completion state.
     */
    public Trigger done(int cyclesToDelay) {
        BooleanSupplier delayFinished = new BooleanSupplier() {
            boolean initialInactive = false;
            int targetPollCount;

            @Override
            public boolean getAsBoolean() {
                if (!AutoTrigger.this.isFinished) {
                    initialInactive = false;
                    return false;
                } else {
                    if (!initialInactive) {
                        initialInactive = true;
                        targetPollCount = AutoTrigger.this.auto.pollCount() + cyclesToDelay;
                    }

                    return AutoTrigger.this.auto.pollCount() == targetPollCount;
                }
            }
        };
        return inactive().and(auto.observe(delayFinished));
    }

    /**
     * Returns a {@link Trigger} that evaluates to true immediately after this trigger finishes.
     *
     * @return A {@link Trigger} representing the immediate completion state.
     */
    public Trigger done() {
        return done(0);
    }

    /**
     * Resets the internal state variables (active, finished, interrupted) to false.
     */
    public void reset() {
        isActive = false;
        isFinished = false;
        interrupted = false;
    }
}
