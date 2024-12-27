package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DriveConsistentHeadingCommand extends Command {
    private final Swerve swerve;
    private final SwerveRequest.FieldCentric fieldCentric;
    private final PIDController headingController;
    private final DoubleConsumer setConsistentHeading;
    private final DoubleSupplier consistentHeading;
    private final DoubleSupplier straight;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rot;

    public DriveConsistentHeadingCommand(
            Swerve swerve,
            SwerveRequest.FieldCentric fieldCentric,
            PIDController headingController,
            DoubleConsumer setConsistentHeading,
            DoubleSupplier consistentHeading,
            DoubleSupplier straight,
            DoubleSupplier strafe,
            DoubleSupplier rot
    ) {
        this.swerve = swerve;
        this.fieldCentric = fieldCentric;
        this.headingController = headingController;
        this.setConsistentHeading = setConsistentHeading;
        this.consistentHeading = consistentHeading;
        this.straight = straight;
        this.strafe = strafe;
        this.rot = rot;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (Math.abs(rot.getAsDouble()) >= 0.1 || (Math.abs(straight.getAsDouble()) < 0.1 && Math.abs(strafe.getAsDouble()) < 0.1)) {
            setConsistentHeading.accept(swerve.localizer.getStrategyPose().getRotation().getDegrees());
            swerve.setControl(
                    fieldCentric.withDeadband(Constants.MAX_VEL * 0.1)
                            .withRotationalDeadband(Constants.MAX_ANGULAR_VEL * 0.1) // Add a 10% deadband
                            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
                            .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL) // Drive forward with negative Y (forward)
                            .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL) // Drive left with negative X (left)
                            .withRotationalRate(-rot.getAsDouble() * Constants.MAX_ANGULAR_VEL) // Drive counterclockwise with negative X (left)
            );
        } else {
            swerve.setControl(
                    fieldCentric.withDeadband(Constants.MAX_VEL * 0.1)
                            .withRotationalDeadband(Constants.MAX_ANGULAR_VEL * 0.1)
                            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                            .withVelocityX(-straight.getAsDouble() * Constants.MAX_VEL)
                            .withVelocityY(-strafe.getAsDouble() * Constants.MAX_VEL)
                            .withRotationalRate(headingController.calculate(
                                    swerve.localizer.getStrategyPose().getRotation().getDegrees(),
                                    consistentHeading.getAsDouble()
                            ) * Constants.MAX_ANGULAR_VEL)
            );
        }
    }
}
