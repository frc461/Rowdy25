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

/**
 * Utility class for various mathematical equations commonly used in control systems.
 *
 * <p> Different mathematical models can provide for different error-based response characteristics.
 * <p> If you need to tune your constants to apply to the exponential function, use <a href="https://www.desmos.com/calculator/yknxk8el8y">this link.</a> Useful for smooth, logistic curves.
 *
 * @author Eugene Zhang, <a href="https://github.com/e500">GitHub</a>
 */
public final class EquationUtil {

    /**
     * Output based on an exponential/logistic function.
     *
     * <p> All numbers should be positive so output decreases as error decreases to zero
     *
     * @param error Input error
     * @param max Maximum output value
     * @param halfway The error value at which the output is half of max
     * @param multiplier Steepness/sensitivity of the curve
     * @return Output value based on error and the specified exponential function
     */
    public static double expOutput(double error, double max, double halfway, double multiplier) {
        return max / (1 + Math.exp(-multiplier * (error - halfway)));
    }

    /**
     * Output based on an exponential/logistic function with max = 1.
     *
     * @param error Input error
     * @param halfway The error value at which the output is half of max
     * @param multiplier Steepness/sensitivity of the curve
     * @return Output value based on error and the specified exponential function
     */
    public static double expOutput(double error, double halfway, double multiplier) {
        return expOutput(error, 1, halfway, multiplier);
    }

    /**
     * Output based on a linear function.
     *
     * @param error Input error
     * @param kP Proportional constant
     * @param offset Constant offset added to the output
     * @return Output value based on error and the specified linear function
     */
    public static double linearOutput(double error, double kP, double offset) {
        return kP * error + offset;
    }

    /**
     * Output based on a linear function with no offset.
     *
     * @param error Input error
     * @param kP Proportional constant
     * @return Output value based on error and the specified linear function
     */
    public static double linearOutput(double error, double kP) { // An inline kP controller
        return linearOutput(error, kP, 0);
    }

    /**
     * Output based on a polynomial function.
     *
     * @param error Input error
     * @param power Power to which the error is raised
     * @param offset Constant offset added to the output
     * @return Output value based on error and the specified polynomial function
     */
    public static double polyOutput(double error, double power, double offset) {
        return Math.pow(error, power) + offset;
    }

    /**
     * Output based on a polynomial function with no offset.
     *
     * @param error Input error
     * @param power Power to which the error is raised
     * @return Output value based on error and the specified polynomial function
     */
    public static double polyOutput(double error, double power) {
        return polyOutput(error, power, 0);
    }
}
