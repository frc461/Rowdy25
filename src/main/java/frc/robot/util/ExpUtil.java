package frc.robot.util;

/* Useful for smooth, logistic curves */
public class ExpUtil {
    /* If you need to tune your constants to apply to the exponential function, here's the link: https://www.desmos.com/calculator/iynksu5yxs */

    public static double output(double error, double halfway, double multiplier) {
        // all numbers should be positive so output decreases as error decreases to zero
        return 1 / (1 + Math.exp(-multiplier * (error - halfway)));
    }
}
