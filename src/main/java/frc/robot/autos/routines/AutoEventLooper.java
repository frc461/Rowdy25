package frc.robot.autos.routines;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * An object that represents an autonomous start or dynamic path.
 *
 * <p>This loop is used to handle autonomous trigger logic and schedule commands. This loop should
 * **not** be shared across multiple autonomous routines.
 *
 * @see frc.robot.Robot Your Mom
 */
public class AutoEventLooper {

    /** The underlying {@link EventLoop} that triggers are bound to and polled */
    private final EventLoop loop;

    /** The name of the auto this loop is associated with */
    private final String name;

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

    public AutoTrigger addTrigger(String name, Command command) {
        return new AutoTrigger(name, command, this);
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
     * Creates a command that will poll this event loop and reset it when it is cancelled.
     *
     * <p>The command will end instantly and kill the routine if the alliance supplier returns an
     * empty optional when the command is scheduled.
     *
     * @return A command that will poll this event loop and reset it when it is cancelled.
     * @see #cmd(BooleanSupplier) A version of this method that takes a condition to finish the loop.
     */
    public Command cmd() {
        return cmd(() -> false);
    }

    /**
     * Creates a command that will poll this event loop and reset it when it is finished or canceled.
     *
     * <p>The command will end instantly and kill the routine if the alliance supplier returns an
     * empty optional when the command is scheduled.
     *
     * @param finishCondition A condition that will finish the loop when it is true.
     * @return A command that will poll this event loop and reset it when it is finished or canceled.
     * @see #cmd() A version of this method that doesn't take a condition and never finishes except if
     *     the alliance supplier returns an empty optional when scheduled.
     */
    public Command cmd(BooleanSupplier finishCondition) {
        return Commands.run(this::poll)
                .finallyDo(this::reset)
                .until(() -> !DriverStation.isAutonomousEnabled() || finishCondition.getAsBoolean())
                .withName(name);
    }
}
