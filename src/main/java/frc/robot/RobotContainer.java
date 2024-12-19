// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.Swerve;

public class RobotContainer {
    /* Subsystems */
    public final Swerve swerve = Constants.SwerveConstants.createDrivetrain();

    private final CommandXboxController driverXbox = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            // TODO MOVE TO METHOD IN SWERVE
            swerve.applyRequest(() ->
                swerve.fieldCentric.withVelocityX(-driverXbox.getLeftY() * Constants.MAX_VEL) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftX() * Constants.MAX_VEL) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * Constants.MAX_ANGULAR_VEL) // Drive counterclockwise with negative X (left)
            )
        );

        driverXbox.a().whileTrue(swerve.applyRequest(() -> swerve.brake));
        driverXbox.b().whileTrue(swerve.applyRequest(() ->
            swerve.point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        ));

        // reset the field-centric heading on y press
        driverXbox.y().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverXbox.back().and(driverXbox.y()).whileTrue(swerve.getSysID().sysIdDynamic(Direction.kForward));
        driverXbox.back().and(driverXbox.x()).whileTrue(swerve.getSysID().sysIdDynamic(Direction.kReverse));
        driverXbox.start().and(driverXbox.y()).whileTrue(swerve.getSysID().sysIdQuasistatic(Direction.kForward));
        driverXbox.start().and(driverXbox.x()).whileTrue(swerve.getSysID().sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
