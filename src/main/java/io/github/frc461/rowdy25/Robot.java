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

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.TimedRobot;
import io.github.frc461.rowdy25.constants.RobotIdentity;

/**
 * Robot implements the {@link TimedRobot} routined robot program framework.
 *
 * <p>The Robot class is the base class by which developers add onto the {@link TimedRobot} framework to customize functionality, that is, mechanism characterization, robot control, automation.</p>
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 */
public class Robot extends TimedRobot {
    /**
     * The command to run at the start of the autonomous period. Note that instead of a timed routine (Any {@link edu.wpi.first.wpilibj.Timer}-based class), this command is required for the autonomous period because of the command-based configuration of this project.
     */
    private Command autonomousCommand;

    /**
     * The {@link RobotContainer} is a fundamental class to initiate functionality. The instance would contain all functionality for the robot, including subsystems, commands, and control.
     */
    private final RobotContainer robotContainer;

    /**
     * Constructor for {@link Robot}. Initialize constants based on the MAC-address identity. Forward the local TCP ports for PhotonVision and Limelight configs to a remote host or port.
     */
    public Robot() {
        RobotIdentity.initializeConstants();
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        robotContainer = new RobotContainer();
    }

    /**
     * This method is called once per loop upon deployment.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.periodic();
    }

    /**
     * This method is called once when the robot enters the disabled period.
     */
    @Override
    public void disabledInit() {}

    /**
     * This method is called once per loop while the robot is in the disabled period.
     */
    @Override
    public void disabledPeriodic() {}

    /**
     * This method is called once when the robot exits the disabled period.
     */
    @Override
    public void disabledExit() {}

    /**
     * This method is called once when the robot enters the autonomous period.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This method is called once per loop while the robot is in the autonomous period.
     */
    @Override
    public void autonomousPeriodic() {}

    /**
     * This method is called once when the robot exits the autonomous period.
     */
    @Override
    public void autonomousExit() {}

    /**
     * This method is called once when the robot enters the teleoperated period.
     */
    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This method is called once per loop while the robot is in the teleoperated period.
     */
    @Override
    public void teleopPeriodic() {}

    /**
     * This method is called once when the robot exits the teleoperated period.
     */
    @Override
    public void teleopExit() {}

    /**
     * This method is called once when the robot enters the test period.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method is called once per loop while the robot is in the test period.
     */
    @Override
    public void testPeriodic() {}

    /**
     * This method is called once when the robot exits the test period.
     */
    @Override
    public void testExit() {}

    /**
     * This method is called once per loop during Java simulation (Gradle).
     */
    @Override
    public void simulationPeriodic() {}
}
