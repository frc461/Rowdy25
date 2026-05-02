package io.github.frc461.rowdy25.util;

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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

/**
 * Utility class for creating a Trigger that toggles true when a given condition is true twice in succession within a specified time threshold.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class DoubleTrueTrigger {

    /**
     * Creates a Trigger that toggles true when the provided condition is true twice in succession within the specified time threshold.
     *
     * @param condition     The BooleanSupplier condition with which check for trueness twice in succession.
     * @param timeThreshold The time threshold in seconds within which the condition must be true twice.
     * @return A Trigger that activates on the double true condition.
     */
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
