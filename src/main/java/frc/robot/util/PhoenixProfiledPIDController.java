package frc.robot.util;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PhoenixProfiledPIDController {
    private final PhoenixPIDController controller;
    private double minInput;
    private double maxInput;
    private double lastTimestamp;

    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public PhoenixProfiledPIDController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        controller = new PhoenixPIDController(Kp, Ki, Kd);
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return controller.atSetpoint();
    }

    public boolean atGoal() {
        return atSetpoint() && goal.equals(setpoint);
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
        profile = new TrapezoidProfile(this.constraints);
    }

    public void setPID(double Kp, double Ki, double Kd) {
        controller.setPID(Kp, Ki, Kd);
    }

    public void setIZone(double iZone) {
        controller.setIZone(iZone);
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        controller.enableContinuousInput(minimumInput, maximumInput);
        minInput = minimumInput;
        maxInput = maximumInput;
    }

    public void disableContinuousInput() {
        controller.disableContinuousInput();
    }

    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        controller.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    public void setTolerance(double positionalTolerance) {
        controller.setTolerance(positionalTolerance, Double.POSITIVE_INFINITY);
    }

    public double calculate(double measurement, double currentTimestamp) {
        if (controller.isContinuousInputEnabled()) {
            // Get error which is the smallest distance between goal and measurement
            double errorBound = (maxInput - minInput) / 2.0;
            double goalMinDistance = MathUtil.inputModulus(goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance =
                    MathUtil.inputModulus(setpoint.position - measurement, -errorBound, errorBound);

            // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
            // may be outside the input range after this operation, but that's OK because the controller
            // will still go there and report an error of zero. In other words, the setpoint only needs to
            // be offset from the measurement by the input range modulus; they don't need to be equal.
            goal.position = goalMinDistance + measurement;
            setpoint.position = setpointMinDistance + measurement;
        }

        double thisPeriod = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        setpoint = profile.calculate(thisPeriod, setpoint, goal);
        return controller.calculate(measurement, setpoint.position, currentTimestamp);
    }

    public double calculate(double measurement, TrapezoidProfile.State goal, double currentTimestamp) {
        setGoal(goal);
        return calculate(measurement, currentTimestamp);
    }

    public double calculate(double measurement, double targetPosition, double currentTimestamp) {
        setGoal(new TrapezoidProfile.State(targetPosition, 0));
        return calculate(measurement, currentTimestamp);
    }

    public double calculate(double measurement, TrapezoidProfile.Constraints constraints, double currentTimestamp) {
        setConstraints(constraints);
        return calculate(measurement, currentTimestamp);
    }

    public double calculate(
            double measurement,
            TrapezoidProfile.State goal,
            TrapezoidProfile.Constraints constraints,
            double currentTimestamp
    ) {
        setConstraints(constraints);
        return calculate(measurement, goal, currentTimestamp);
    }

    public void reset(TrapezoidProfile.State measurement, double timestamp) {
        controller.reset();
        setpoint = measurement;
        lastTimestamp = timestamp;
    }

    // In the case of the goal being set to 0, currentPosition is the current error and the currentVelocity is the change in error
    public void reset(double currentPosition, double currentVelocity, double timestamp) {
        reset(new TrapezoidProfile.State(currentPosition, currentVelocity), timestamp);
    }

    public void reset(double measuredPosition, double timestamp) {
        reset(measuredPosition, 0.0, timestamp);
    }
}
