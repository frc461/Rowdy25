// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.SysID;
import frc.robot.util.VisionUtil;

public class RobotContainer {
    /* Subsystems */
    public final Swerve swerve = new Swerve();
    private final AutoFactory autoFactory;

    /* Sys ID */
    public final SysID sysID = new SysID(swerve);

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    /* Currently Allocated For Driver:
     * POV buttons / D-pad:
     * Up:
     * Down:
     * Left:
     * Right:
     *
     * Triggers:
     * Left: align
     * Right:
     *
     * Joysticks:
     * Left: Translation
     * Right: Rotation
     * Left Button:
     * Right Button:
     *
     * Bumpers:
     * Left:
     * Right:
     *
     * Buttons:
     * A: X mode
     * B: Point directions
     * X:
     * Y: Re-zero gyro
     */

    public final static CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / D-pad:
     * Up:
     * Down:
     * Left:
     * Right:
     *
     * Triggers:
     * Left:
     * Right:
     *
     * Joysticks:
     * Left:
     * Right:
     *
     * Bumpers:
     * Left:
     * Right:
     *
     * Buttons:
     * A:
     * B:
     * X:
     * Y:
     */

    public RobotContainer() {
        setDefaultCommands();
        configureBindings();

        autoFactory = new AutoFactory(
            swerve.getLocalizer()::getEstimatedPose, // A function that returns the current robot pose
            swerve.getLocalizer()::setPoses, // A function that resets the current robot pose to the provided Pose2d
            swerve::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerve, // The drive subsystem
            new AutoBindings() // An empty AutoBindings object 
        );
    }

    /* Each subsystem will execute their corresponding command periodically */
    private void setDefaultCommands() {
        /* Note that X is defined as forward according to WPILib convention,
        and Y is defined as to the left according to WPILib convention.
        drive forward with left joystick negative Y (forward),
        drive left with left joystick negative X (left),
        rotate counterclockwise with right joystick negative X (left) */
        swerve.setDefaultCommand(
                swerve.driveFieldCentric(
                        driverXbox::getLeftY,
                        driverXbox::getLeftX,
                        driverXbox::getRightX
                )
        );
    }

    private void configureBindings() {

        driverXbox.a().whileTrue(swerve.xMode());

        // toggle between robot choosing quest nav pose and pose estimation with cameras
        driverXbox.b().onTrue(swerve.runOnce(swerve::switchLocalizationMode));

        driverXbox.x().onTrue(swerve.runOnce(swerve::recalibrate));

        // reset the field-centric heading on y press
        driverXbox.y().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        driverXbox.leftBumper().whileTrue(swerve.driveTurret(driverXbox::getLeftY, driverXbox::getLeftX));
        driverXbox.rightBumper().whileTrue(swerve.centerOnNote(driverXbox::getLeftY, driverXbox::getLeftX));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(driverXbox);
    }

    public Command getAutonomousCommand() {
        AutoRoutine autoRoutine = autoFactory.newRoutine("branched");
        AutoTrajectory start = autoRoutine.trajectory("start");
        AutoTrajectory end1 = autoRoutine.trajectory("end1");
        AutoTrajectory end2 = autoRoutine.trajectory("end2");

        autoRoutine.active().onTrue(
                Commands.sequence(
                    autoRoutine.resetOdometry(start),
                    start.cmd()
                )
        );

        start.done().onTrue(VisionUtil.Photon.Color.hasTargets() ? end1.cmd() : end2.cmd());

        return autoRoutine.cmd();
    }

}
