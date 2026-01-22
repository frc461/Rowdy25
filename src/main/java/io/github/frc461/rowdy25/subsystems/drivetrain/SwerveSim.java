package io.github.frc461.rowdy25.subsystems.drivetrain;

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

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import io.github.frc461.rowdy25.subsystems.localizer.LocalizerSim;

public class SwerveSim {
    private final Swerve swerve;
    private final LocalizerSim localizerSim = new LocalizerSim();

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private double lastSimTime;

    public SwerveSim(Swerve swerve) {
        this.swerve = swerve;
    }

    public void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        try (Notifier notifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;

                /* use the measured time delta, get battery voltage from WPILib */
                swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage());
                localizerSim.update(swerve.getState().Pose);
        })) {
            notifier.startPeriodic(SIM_LOOP_PERIOD);
        }
    }
}
