package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class AutoTrigger {
    private final String name;

    private final Command triggeredCommand;

    private final AutoEventLooper auto;

    private boolean isActive = false;

    private boolean isFinished = false;

    public AutoTrigger(String name, Command path, AutoEventLooper auto) {
        this.name = name;
        this.triggeredCommand = path;
        this.auto = auto;
    }

    public Command cmd() {
        return new WrapperCommand(this.triggeredCommand) {
            @Override
            public void initialize() {
                super.initialize();
                isActive = true;
                isFinished = false;
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                isActive = false;
                isFinished = !interrupted;
            }

            @Override
            public boolean isFinished() {
                return super.isFinished() || !auto.isActive;
            }
        }.withName(name);
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

    public Trigger done(int cyclesToDelay) {
        BooleanSupplier delayFinished = new BooleanSupplier() {
            boolean initialInactive = false;
            int targetPollCount;

            @Override
            public boolean getAsBoolean() {
                // TODO DETERMINE IF THIS REQUIRES THE PATH TO BE ACTUALLY COMPLETED OR MAY BE STOPPED
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
