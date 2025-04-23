package io.github.frc461.rowdy25.autos.routines;

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
