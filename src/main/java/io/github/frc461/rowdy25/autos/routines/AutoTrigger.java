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

public class AutoTrigger {
    public final String name;
    private final AutoEventLooper auto;
    private final Supplier<Command> triggeredCommand;
    private Command decoratedCommand = null;

    public boolean isActive = false;
    public boolean isFinished = false;
    public boolean interrupted = false;

    public AutoTrigger(String name, Supplier<Command> command, AutoEventLooper auto) {
        this.name = name;
        this.auto = auto;
        this.triggeredCommand = command;
    }

    public AutoTrigger duplicate() {
        return new AutoTrigger(this.name, this.triggeredCommand, this.auto);
    }

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
     * Returns a {@link Trigger} that is true while this autonomous routine is being polled.
     *
     * <p>Using a {@link Trigger#onFalse(Command)} will do nothing as when this is false the routine
     * is not being polled anymore.
     *
     * @return A {@link Trigger} that is true while this autonomous routine is being polled.
     */
    public Trigger active() {
        return auto.observe(() -> isActive && auto.isActive);
    }

    public Trigger inactive() {
        return active().negate();
    }

    public Trigger interrupt() {
        return inactive().and(auto.observe(() -> interrupted));
    }

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

    public Trigger done() {
        return done(0);
    }

    public void reset() {
        isActive = false;
        isFinished = false;
        interrupted = false;
    }
}
