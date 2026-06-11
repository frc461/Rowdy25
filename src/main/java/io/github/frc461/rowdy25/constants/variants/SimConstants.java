package io.github.frc461.rowdy25.constants.variants;

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
 * Robot-specific constant overrides for simulation.
 * <p>
 * These values override the default angular velocity D gains for swerve control
 * when running in simulation mode, where physical damping is not present.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class SimConstants {
    /** Angular position D gain for yaw control in simulation (no physical damping). */
    public final static double ANGULAR_POSITION_D = 0.0;

    /** Angular object detection D gain for simulation (no physical damping). */
    public final static double ANGULAR_OBJECT_DETECTION_D = 0.00;
}