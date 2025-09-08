/**
 * Main execution file.
 *
 * <p>Copyright (c) 2025 461 Boosters First, Inc. dba Westside Robotics - The Rowdy 25.</p>
 *
 * @author Eugene Zhang, https://github.com/e500
 * @author Aneesh Terani, https://github.com/aterani
 */
package io.github.frc461.rowdy25;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class initiates the robot instance.
 *
 * <p>The conventional Java main method executes as usual after project deployment.
 */
public final class Main {
    /**
     * Constructor for {@link Main}. Remains for formality.
     */
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
