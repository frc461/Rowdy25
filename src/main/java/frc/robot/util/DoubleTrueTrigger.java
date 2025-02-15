package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

public class DoubleTrueTrigger {
    public static Trigger doubleTrue(BooleanSupplier condition, double timeThreshold) {
        return new Trigger(
                new BooleanSupplier() {
                    final Debouncer timeoutDebouncer = new Debouncer(timeThreshold);
                    boolean initialClick = false;
                    boolean initialRelease = false;
                    boolean doubleClicked = false;

                    @Override
                    public boolean getAsBoolean() {
                        if (!doubleClicked && timeoutDebouncer.calculate(initialClick)) { // upon time out after initially clicked
                            initialClick = false;
                            initialRelease = false;
                        } else if (!initialClick) {
                            initialClick = condition.getAsBoolean(); // upon initial trigger activation
                        } else if (!initialRelease) {
                            initialRelease = !condition.getAsBoolean(); // upon initial trigger deactivation
                        } else { // after first trigger cycle
                                doubleClicked = condition.getAsBoolean();
                                initialClick = doubleClicked || initialClick;
                                initialRelease = doubleClicked || initialRelease;
                        }
                        return doubleClicked;
                    }
                }
        );
    }
}
