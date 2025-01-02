package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.FollowPathDynamicCommand;

import java.util.function.BooleanSupplier;

public class AutoDynamicPathTrigger {
    private final String name;

    private final AutoEventLooper auto;

    private final FollowPathDynamicCommand triggeredPath;

    private boolean isActive = false;

    private boolean isFinished = false;

    private boolean interrupted = false;

    public AutoDynamicPathTrigger(String name, FollowPathDynamicCommand path, AutoEventLooper auto) {
        this.name = name;
        this.auto = auto;
        this.triggeredPath = path;
    }

    public Command cmd() {
        return triggeredPath.finallyDo(
                interrupted -> {
                    isActive = false;
                    isFinished = !interrupted;
                    AutoDynamicPathTrigger.this.interrupted = interrupted;
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
                if (!AutoDynamicPathTrigger.this.isFinished) {
                    initialInactive = false;
                    return false;
                } else {
                    if (!initialInactive) {
                        initialInactive = true;
                        targetPollCount = AutoDynamicPathTrigger.this.auto.pollCount() + cyclesToDelay;
                    }

                    return AutoDynamicPathTrigger.this.auto.pollCount() == targetPollCount;
                }
            }
        };
        return inactive().and(auto.observe(delayFinished));
    }

    public Trigger done() {
        return done(0);
    }
}
