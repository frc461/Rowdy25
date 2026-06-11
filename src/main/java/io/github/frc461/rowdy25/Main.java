package io.github.frc461.rowdy25;

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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class initiates and executes the robot instance.
 *
 * <p>The Java main method is the entry point of program execution after project deployment.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 */
public final class Main {
    /**
     * Constructor for {@link Main}. Declared private to prevent instantiation.
     */
    private Main() {}

    /**
     * Initiates the robot instance.
     *
     * @param args Custom arguments passed into the main method. Not utilized in this program.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
