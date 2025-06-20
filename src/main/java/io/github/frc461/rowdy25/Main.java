package io.github.frc461.rowdy25;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class initiates the robot instance.
 *
 * <p>The conventional Java main method executes as usual after project deployment.
 */
public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
