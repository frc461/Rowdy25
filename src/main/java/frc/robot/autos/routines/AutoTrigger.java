package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class AutoTrigger {
    private final String name;
    private final AutoEventLooper auto;
    private final Command triggeredCommand;

    private boolean isActive = false;
    private boolean isFinished = false;
    private boolean interrupted = false;

    public AutoTrigger(String name, Command command, AutoEventLooper auto) {
        this.name = name;
        this.auto = auto;
        this.triggeredCommand = command;
    }

    public Command cmd() {
        return triggeredCommand.finallyDo(
                interrupted -> {
                    isActive = false;
                    isFinished = !interrupted;
                    AutoTrigger.this.interrupted = interrupted;
                }
        ).beforeStarting(
                () -> {
                    isActive = true;
                    isFinished = false;
                    interrupted = false;
                }
        ).withName(name);
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
}
