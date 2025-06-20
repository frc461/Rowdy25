package io.github.frc461.rowdy25;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.TimedRobot;
import io.github.frc461.rowdy25.constants.RobotIdentity;

/**
 * Robot implements the TimedRobot routined robot program framework.
 *
 * <p>The Robot class is the base class by which developers add onto the framework to customize functionality i.e., mechanism characterization, robot control, automation.
 *
 */
public class Robot extends TimedRobot {
    /**
     * The command to run at the start of the autonomous period. Note that instead of a timed routine, this command is required for the autonomous period because of the command-based configuration of this project.
     */
    private Command autonomousCommand;

    /**
     * The RobotContainer is the primary class structure used to develop robot functionality. The instance contains all functionality for the robot, including subsystems, commands, and control.
     */
    private final RobotContainer robotContainer;

    /**
     * Constructor for Robot. Initialize constants based on the MAC-address identity. Forward the local TCP ports for PhotonVision and Limelight configs to a remote host or port.
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
