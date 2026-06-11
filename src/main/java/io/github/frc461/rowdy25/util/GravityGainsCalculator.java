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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Utility class to calculate gravity compensation gains for a robotic arm with a pivoting joint, a wrist joint, and an elevator mechanism.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public final class GravityGainsCalculator {
    /** The position of the pivot axis in a 2D plane. */
    private final Translation2d pivotAxisPosition;
    /** The position of the wrist axis in a 2D plane. */
    private final Translation2d wristAxisPosition;
    /** The vector from the wrist axis to the zero position (default/stowed) center of mass. */
    private final Translation2d wristAxisToZeroCoM;
    /** The position of the elevator's center of mass with a zeroed elevator and uprightly/perpendicularly positioned pivot. */
    private final Translation2d elevatorZeroUprightCoM;
    /** The ratio of the elevator's center of mass movement to stage 2 elevator movement. */
    private final double elevatorCoMToStage2Ratio;
    /** The limit position of stage 3 of the elevator (in inches). */
    private final double elevatorStage3Limit;
    /** The ratio of the elevator's center of mass movement to stage 3 elevator movement. */
    private final double elevatorCoMToStage3Ratio;
    /** The mass of the elevator in pounds. */
    private final double elevatorMassLbs;
    /** The mass of the wrist in pounds. */
    private final double wristMassLbs;
    /** The base gravity gain constant to be tuned. */
    private final double kG;

    /** The base length from the pivot axis to the center of mass with a zeroed elevator and uprightly/perpendicularly positioned pivot. */
    private final double baseLengthPivotAxisToZeroCoM;

    /**
     * Constructs a GravityGainsCalculator with the specified parameters.
     *
     * @param pivotAxisPosition The position of the pivot axis in a 2D plane.
     * @param wristAxisPosition The position of the wrist axis in a 2D plane
     * @param wristAxisToZeroCoM The vector from the wrist axis to the zero position (default/stowed) center of mass.
     * @param elevatorZeroUprightCoM The position of the elevator's center of mass with a zeroed elevator and uprightly/perpendicularly positioned pivot.
     * @param elevatorCoMToStage2Ratio The ratio of the elevator's center of mass movement to stage 2 elevator movement.
     * @param elevatorStage3Limit The limit position of stage 3 of the elevator (in inches).
     * @param elevatorCoMToStage3Ratio The ratio of the elevator's center of mass movement to stage 3 elevator movement.
     * @param elevatorMassLbs The mass of the elevator in pounds.
     * @param wristMassLbs The mass of the wrist in pounds.
     * @param kG The base gravity gain constant to be tuned.
     *
     */
    public GravityGainsCalculator(
        Translation2d pivotAxisPosition,
        Translation2d wristAxisPosition,
        Translation2d wristAxisToZeroCoM,
        Translation2d elevatorZeroUprightCoM,
        double elevatorCoMToStage2Ratio,
        double elevatorStage3Limit,
        double elevatorCoMToStage3Ratio,
        double elevatorMassLbs,
        double wristMassLbs,
        double kG
    ) {
        this.pivotAxisPosition = pivotAxisPosition;
        this.wristAxisPosition = wristAxisPosition;
        this.wristAxisToZeroCoM = wristAxisToZeroCoM;
        this.elevatorZeroUprightCoM = elevatorZeroUprightCoM;
        this.elevatorCoMToStage2Ratio = elevatorCoMToStage2Ratio;
        this.elevatorStage3Limit = elevatorStage3Limit;
        this.elevatorCoMToStage3Ratio = elevatorCoMToStage3Ratio;
        this.elevatorMassLbs = elevatorMassLbs;
        this.wristMassLbs = wristMassLbs;
        this.kG = kG;

        Translation2d zeroCoM = elevatorZeroUprightCoM.times(elevatorMassLbs).plus((wristAxisPosition.plus(wristAxisToZeroCoM).times(wristMassLbs))).div(wristMassLbs + elevatorMassLbs);
        this.baseLengthPivotAxisToZeroCoM = pivotAxisPosition.getDistance(zeroCoM);
    }

    /**
     * Calculates the gravity compensation gain based on the provided pivot position, wrist position, and elevator position.
     *
     * @param pivotPosition The angle of the pivot joint in degrees.
     * @param wristPosition The angle of the wrist joint in degrees.
     * @param elevatorPosition The position of the elevator in inches.
     * @return The calculated gravity compensation gain.
     *
     */
    public double calculateGFromPositions(
        double pivotPosition,
        double wristPosition,
        double elevatorPosition
    ) {
        Translation2d wristAxisElevatedPosition = wristAxisPosition.plus(new Translation2d(0, elevatorPosition));
        Translation2d pivotToWristAxes = wristAxisElevatedPosition.minus(pivotAxisPosition);
        Translation2d newWristAxis = pivotAxisPosition.plus(pivotToWristAxes.rotateBy(Rotation2d.fromDegrees(-(90 - pivotPosition))));

        Translation2d currentWristAxisToCoM = wristAxisToZeroCoM.rotateBy(Rotation2d.fromDegrees(wristPosition));
        Translation2d currentWristCoM = newWristAxis.plus(currentWristAxisToCoM);

        Translation2d currentElevatorUprightCoM = elevatorZeroUprightCoM.plus(
                new Translation2d(0, elevatorCoMToStage3Ratio * Math.min(elevatorStage3Limit, elevatorPosition)
                        + elevatorCoMToStage2Ratio * Math.max(0, elevatorPosition - elevatorStage3Limit))
        );
        Translation2d currentElevatorCoM = pivotAxisPosition.plus(currentElevatorUprightCoM.minus(pivotAxisPosition).rotateBy(Rotation2d.fromDegrees(-(90 - pivotPosition))));

        Translation2d currentCoM = currentWristCoM.times(wristMassLbs).plus(currentElevatorCoM.times(elevatorMassLbs)).div(wristMassLbs + elevatorMassLbs);

        double lengthPivotAxisToCoM = pivotAxisPosition.getDistance(currentCoM);

        return kG * Math.cos(Math.toRadians(pivotPosition)) * (lengthPivotAxisToCoM / baseLengthPivotAxisToZeroCoM);
    }

    /** Test module for the GravityGainsCalculator class. */
    public static void main(String[] args) {

        Translation2d pivotAxisPosition = new Translation2d(-9.417377, 9.257139); // CONSTANT
        Translation2d wristAxisPosition = new Translation2d(-11.767377, 38.007139); // CONSTANT
        Translation2d wristAxisToZeroCoM = new Translation2d(0, -7.453525); // CONSTANT
        Translation2d elevatorZeroUprightCoM = new Translation2d(-11.347053, 15.125012); // CONSTANT
        double elevatorCoMToStage2Ratio = 0.509767; // CONSTANT
        double elevatorStage2Limit = 24; // CONSTANT
        double elevatorCoMToStage3Ratio = 0.3345002; // CONSTANT
        double elevatorMass = 23.0132625; // CONSTANT
        double wristMass = 7.1301147; // CONSTANT
        double kG = 0.2269; // CONSTANT

        GravityGainsCalculator calculator = new GravityGainsCalculator(
            pivotAxisPosition,
            wristAxisPosition,
            wristAxisToZeroCoM,
            elevatorZeroUprightCoM,
            elevatorCoMToStage2Ratio,
            elevatorStage2Limit,
            elevatorCoMToStage3Ratio,
            elevatorMass,
            wristMass,
            kG
        );

        System.out.println(calculator.calculateGFromPositions(45, 120, 5));
    }
}
