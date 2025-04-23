package io.github.frc461.rowdy25.util;

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
