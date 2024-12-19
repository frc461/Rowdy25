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
    private final Telemetry logger = new Telemetry(Constants.MAX_VEL);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final Swerve swerve = Constants.SwerveConstants.createDrivetrain();

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
                swerve.fieldCentric.withVelocityX(-joystick.getLeftY() * Constants.MAX_VEL) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.MAX_VEL) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.MAX_ANGULAR_VEL) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(swerve.applyRequest(() -> swerve.brake));
        joystick.b().whileTrue(swerve.applyRequest(() ->
            swerve.point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // reset the field-centric heading on y press
        joystick.y().onTrue(swerve.runOnce(swerve::seedFieldCentric));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(swerve.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(swerve.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(swerve.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(swerve.sysIdQuasistatic(Direction.kReverse));

        swerve.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
