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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.frc461.rowdy25.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * A polling event loop that scaffolds a dynamic autonomous routine.
 *
 * <p>The looper owns a single private {@link EventLoop} and a list of {@link AutoTrigger}s. Its
 * {@link #cmd()} method returns a long-running {@link Command} that, on every scheduler tick,
 * calls {@link EventLoop#poll()} to re-evaluate every {@link Trigger} bound to it. Trigger rising
 * edges (e.g., the {@code done()} trigger of one segment) schedule subsequent segments, which
 * yields the chained behavior used by {@code AutoManager.generateAutoEventLooper(...)}.
 *
 * <p>A given {@code AutoEventLooper} instance owns its own loop state and <strong>must not</strong>
 * be shared across multiple concurrent autonomous routines.
 *
 * @see AutoTrigger
 * @see Robot
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class AutoEventLooper {

    /** The underlying {@link EventLoop} that triggers are bound to and polled */
    private final EventLoop loop;

    /** The name of the auto this loop is associated with */
    private final String name;

    private final List<AutoTrigger> triggers;

    /** A boolean utilized in {@link #active()} to resolve trueness */
    protected boolean isActive = false;

    /** A boolean that is true when the loop is killed */
    private boolean isKilled = false;

    /** The amount of times the routine has been polled */
    private int pollCount = 0;

    /**
     * A constructor to be used when inheriting this class to instantiate a custom inner loop
     *
     * @param name The name of the loop
     * @param loop The inner {@link EventLoop}
     */
    public AutoEventLooper(String name, EventLoop loop) {
        this.loop = loop;
        this.name = name;
        triggers = new ArrayList<>();
    }

    /**
     * Creates a new loop with a specific name
     *
     * @param name The name of the loop
     */
    public AutoEventLooper(String name) {
        this(name, new EventLoop());
    }

    /**
     * Gets the event loop that this routine is using.
     *
     * @return The event loop that this routine is using.
     */
    public EventLoop loop() {
        return loop;
    }

    /**
     * Gets the poll count of the routine.
     *
     * @return The poll count of the routine.
     */
    public int pollCount() {
        return pollCount;
    }

    /**
     * Creates a {@link Trigger} that is bound to the routine's {@link EventLoop}.
     *
     * @param condition The condition represented by the trigger.
     * @return A {@link Trigger} that mirrors the state of the provided {@code condition}
     */
    public Trigger observe(BooleanSupplier condition) {
        return new Trigger(loop, condition);
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
        return observe(() -> isActive && DriverStation.isAutonomousEnabled());
    }

    /**
     * Registers a new {@link AutoTrigger} on this looper's event loop.
     *
     * <p>The supplied {@link Command} is constructed lazily (only on the first call to
     * {@link AutoTrigger#cmd()}), so it is safe to capture state — such as pose targets — that
     * was resolved when the surrounding routine was compiled.
     *
     * @param name A unique-per-loop label used for telemetry / {@link Command#withName(String)}.
     * @param command Supplier for the command this trigger schedules when fired.
     * @return The created {@link AutoTrigger}, already attached to this looper.
     */
    public AutoTrigger addTrigger(String name, Supplier<Command> command) {
        AutoTrigger trigger = new AutoTrigger(name, command, this);
        triggers.add(trigger);
        return trigger;
    }

    /**
     * Creates a trigger that produces a rising edge when any of the trajectories are finished.
     *
     * @param trajectory The first trajectory to watch.
     * @param trajectories The other trajectories to watch
     * @return a trigger that determines if any of the trajectories are finished
     * @see #anyDone(int, AutoTrigger, AutoTrigger...) A version of this method that takes a
     *     delay in cycles before the trigger is true.
     */
    public Trigger anyDone(AutoTrigger trajectory, AutoTrigger... trajectories) {
        return anyDone(0, trajectory, trajectories);
    }

    /**
     * Creates a trigger that produces a rising edge when any of the paths are finished.
     *
     * @param cyclesToDelay The number of cycles to delay.
     * @param firstPath The first path to watch.
     * @param paths The other paths to watch
     * @return a trigger that determines if any of the paths are finished
     */
    public Trigger anyDone(
            int cyclesToDelay, AutoTrigger firstPath, AutoTrigger... paths) {
        var trigger = firstPath.done(cyclesToDelay);
        for (AutoTrigger path : paths) {
            trigger = trigger.or(path.done(cyclesToDelay));
        }
        return trigger.and(this.active());
    }

    /**
     * Creates a trigger that returns true when any of the paths given are active.
     *
     * @param firstPath The first path to watch.
     * @param paths The other paths to watch
     * @return a trigger that determines if any of the paths are active
     */
    public Trigger anyActive(AutoEventLooper firstPath, AutoEventLooper... paths) {
        var trigger = firstPath.active();
        for (AutoEventLooper path : paths) {
            trigger = trigger.or(path.active());
        }
        return trigger.and(this.active());
    }

    /** Polls the routine. Should be called in the autonomous periodic method. */
    public void poll() {
        if (!DriverStation.isAutonomousEnabled()
                || isKilled) {
            isActive = false;
            return;
        }
        pollCount++;
        loop.poll();
        isActive = true;
    }

    /**
     * Resets the routine. This can either be called on auto init or auto end to reset the routine
     * incase you run it again. If this is called on a routine that doesn't need to be reset it will
     * do nothing.
     */
    public void reset() {
        pollCount = 0;
        isActive = false;
        for (AutoTrigger trigger : triggers) {
            trigger.reset();
        }
    }

    /** Kills the loop and prevents it from running again. */
    public void kill() {
        CommandScheduler.getInstance().cancelAll();
        if (isKilled) {
            return;
        }
        reset();
        isKilled = true;
    }

    /**
     * Creates a command that will poll this event loop every scheduler tick and reset it when canceled
     * or when autonomous ends.
     *
     * @return A command that polls this event loop until autonomous is disabled.
     * @see #cmd(BooleanSupplier) A version of this method that also accepts an external finish condition.
     */
    public Command cmd() {
        return cmd(() -> false);
    }

    /**
     * Creates a command that will poll this event loop every scheduler tick and reset it when the
     * external finish condition becomes true, when autonomous ends, or when the command is canceled.
     *
     * @param finishCondition An additional condition that, when true, finishes the polling command.
     * @return A command that polls this event loop until autonomous is disabled or {@code finishCondition} fires.
     * @see #cmd() A version of this method that polls indefinitely (until autonomous ends).
     */
    public Command cmd(BooleanSupplier finishCondition) {
        return Commands.run(this::poll)
                .finallyDo(this::reset)
                .beforeStarting(this::reset)
                .until(() -> !DriverStation.isAutonomousEnabled() || finishCondition.getAsBoolean())
                .withName(name);
    }
}
