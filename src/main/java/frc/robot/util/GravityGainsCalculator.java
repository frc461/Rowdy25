package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GravityGainsCalculator { // TODO: IMPLEMENT FOR PIVOT GRAVITY GAINS
    public static void main(String[] args) {

        Translation2d pivotAxisPosition = new Translation2d(-9.417377, 9.257139); // CONSTANT
        Translation2d wristAxisPosition = new Translation2d(-11.767377, 38.007139); // CONSTANT
        Translation2d wristZeroAxisToCoM = new Translation2d(0, -7.453525); // CONSTANT
        Translation2d elevatorZeroUprightCoM = new Translation2d(-11.347053, 15.125012); // CONSTANT
        double elevatorCoMIncreaseToStage2IncreaseRatio = 0.509767; // CONSTANT
        double elevatorStage2Extension = 24; // CONSTANT
        double elevatorCoMIncreaseToStage3IncreaseRatio = 0.3345002; // CONSTANT
        double elevatorMass = 23.0132625; // CONSTANT
        double wristMass = 7.1301147; // CONSTANT

        Translation2d zeroCoM = elevatorZeroUprightCoM.times(elevatorMass).plus((wristAxisPosition.plus(wristZeroAxisToCoM).times(wristMass))).div(wristMass + elevatorMass); // CONSTANT
        double pivotAxisToZeroCoM = pivotAxisPosition.getDistance(zeroCoM); // CONSTANT

        double kG = 0.2269; // CONSTANT

        // CHANGE THESE VALUES TO TEST THIS SCRIPT
        double wristPosition = 35; // WRIST CANCODER POSITION
        double elevatorPosition = 28; // ELEVATOR ROTOR POSITION
        double pivotPosition = 16; // PIVOT CANCODER POSITION

        Translation2d wristAxisElevatedPosition = wristAxisPosition.plus(new Translation2d(0, elevatorPosition)); // ELEVATOR ROTOR POSITION
        Translation2d pivotToWristAxes = wristAxisElevatedPosition.minus(pivotAxisPosition);
        Translation2d newWristAxis = pivotAxisPosition.plus(pivotToWristAxes.rotateBy(Rotation2d.fromDegrees(-(90-pivotPosition)))); // 90 MINUS PIVOT CANCODER POSITION CW

        Translation2d currentWristAxisToCoM = wristZeroAxisToCoM.rotateBy(Rotation2d.fromDegrees(wristPosition)); // WRIST CANCODER POSITION
        Translation2d currentWristCoM = newWristAxis.plus(currentWristAxisToCoM);

        Translation2d currentElevatorUprightCoM = elevatorZeroUprightCoM.plus(new Translation2d(0, elevatorCoMIncreaseToStage2IncreaseRatio * Math.min(elevatorStage2Extension, elevatorPosition) + elevatorCoMIncreaseToStage3IncreaseRatio * Math.max(0, elevatorPosition - elevatorStage2Extension))); // ELEVATOR ROTOR POSITION
        Translation2d currentElevatorCoM = pivotAxisPosition.plus(currentElevatorUprightCoM.minus(pivotAxisPosition).rotateBy(Rotation2d.fromDegrees(-(90-pivotPosition)))); // 90 MINUS PIVOT CANCODER POSITION CW

        Translation2d currentCoM = currentWristCoM.times(wristMass).plus(currentElevatorCoM.times(elevatorMass)).div(wristMass + elevatorMass);

        double pivotAxisToCoM = pivotAxisPosition.getDistance(currentCoM);
        double kGToApply = kG * Math.cos(Math.toRadians(pivotPosition)) * (pivotAxisToCoM / pivotAxisToZeroCoM); // PIVOT CANCODER POSITION

        System.out.println(newWristAxis.getX() + " " + newWristAxis.getY());
        System.out.println(currentWristCoM.getX() + " " + currentWristCoM.getY());
        System.out.println(currentElevatorUprightCoM.getX() + " " + currentElevatorUprightCoM.getY());
        System.out.println(currentElevatorCoM.getX() + " " + currentElevatorCoM.getY());
        System.out.println(currentCoM.getX() + " " + currentCoM.getY());
        System.out.println(pivotAxisToCoM);
        System.out.println(pivotAxisToZeroCoM);
        System.out.println(kGToApply);
    }
}
