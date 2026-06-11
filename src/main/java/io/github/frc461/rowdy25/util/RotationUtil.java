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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

/**
 * Utility class for operations involving angles represented by {@link Rotation2d}.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class RotationUtil {
    /**
     * Checks if a given angle is between two other angles, considering the circular nature of angles, assuming continuous measurement.
     *
     * @param angle               The angle to check.
     * @param lowerAngleThreshold The lower bound angle.
     * @param upperAngleThreshold The upper bound angle.
     * @return True if the angle is between the lower and upper bounds, false otherwise.
     */
    public static boolean inBetween(Rotation2d angle, Rotation2d lowerAngleThreshold, Rotation2d upperAngleThreshold) {
        if (lowerAngleThreshold.getDegrees() > upperAngleThreshold.getDegrees()) {
            return angle.getDegrees() > lowerAngleThreshold.getDegrees() || angle.getDegrees() < upperAngleThreshold.getDegrees();
        }
        return angle.getDegrees() > lowerAngleThreshold.getDegrees() && angle.getDegrees() < upperAngleThreshold.getDegrees();
    }

    /**
     * Finds the minimum and maximum angles from a list of angles, considering the circular nature of angles.
     *
     * @param angles The list of angles to evaluate.
     * @return A {@link Pair} containing the minimum and maximum angles.
     */
    public static Pair<Rotation2d, Rotation2d> getBound(List<Rotation2d> angles) {
        Rotation2d min = angles.get(0);
        Rotation2d max = angles.get(0);
        for (Rotation2d angle : angles) {
            if (!inBetween(angle, min, max)) {
                Rotation2d divider = min.interpolate(max, 0.5).rotateBy(Rotation2d.kPi);
                if (inBetween(angle, divider, min)) {
                    min = angle;
                } else {
                    max = angle;
                }
            }
        }
        return new Pair<>(min, max);
    }
}
