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

/* Useful for smooth, logistic curves */
public final class EquationUtil {
    /* If you need to tune your constants to apply to the exponential function, here's the link: https://www.desmos.com/calculator/yknxk8el8y */

    public static double expOutput(double error, double max, double halfway, double multiplier) {
        return max / (1 + Math.exp(-multiplier * (error - halfway)));
    }

    public static double expOutput(double error, double halfway, double multiplier) {
        // all numbers should be positive so output decreases as error decreases to zero
        return expOutput(error, 1, halfway, multiplier);
    }

    public static double linearOutput(double error, double kP, double offset) {
        return kP * error + offset;
    }

    public static double linearOutput(double error, double kP) { // An inline kP controller
        return linearOutput(error, kP, 0);
    }

    public static double polyOutput(double error, double power, double offset) {
        return Math.pow(error, power) + offset;
    }

    public static double polyOutput(double error, double power) {
        return polyOutput(error, power, 0);
    }
}
