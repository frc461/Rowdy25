package frc.robot.util;

/* Useful for smooth, logistic curves */
public final class ExpUtil {
    /* If you need to tune your constants to apply to the exponential function, here's the link: https://www.desmos.com/calculator/yknxk8el8y */

    public static double output(double error, double halfway, double multiplier) {
        // all numbers should be positive so output decreases as error decreases to zero
        return output(error, 1, halfway, multiplier);
    }

    public static double output(double error, double max, double halfway, double multiplier) {
        return max / (1 + Math.exp(-multiplier * (error - halfway)));
    }
}
