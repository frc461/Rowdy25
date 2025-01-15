package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;

public class SwerveSim {
    private final Swerve swerve;

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;

    public SwerveSim(Swerve swerve) {
        this.swerve = swerve;
    }

    public void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;

                /* use the measured time delta, get battery voltage from WPILib */
                swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
        }).startPeriodic(SIM_LOOP_PERIOD);
    }
}
