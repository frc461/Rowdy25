package io.github.frc461.rowdy25.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class GravityGainsCalculator {
    private final Translation2d pivotAxisPosition;
    private final Translation2d wristAxisPosition;
    private final Translation2d wristAxisToZeroCoM;
    private final Translation2d elevatorZeroUprightCoM;
    private final double elevatorCoMToStage2Ratio;
    private final double elevatorStage3Limit;
    private final double elevatorCoMToStage3Ratio;
    private final double elevatorMassLbs;
    private final double wristMassLbs;
    private final double kG;

    private final double baseLengthPivotAxisToZeroCoM;

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
