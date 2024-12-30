// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autos.AutoManager;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.SysID;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
    /* Subsystems */
    public final Swerve swerve = new Swerve();

    private final AutoChooser autoChooser = new AutoChooser();

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
     * Left: "speaker tag" align
     * Right: note align
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
     * X: recalibrate
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
       
        DogLog.setOptions(new DogLogOptions(false, false, true, true, false, 5000));
        DogLog.setPdh(new PowerDistribution());

        Map<String, Supplier<AutoRoutine>> routines = new AutoManager(new AutoFactory(
                swerve.localizer::getStrategyPose, // A function that returns the current robot pose
                swerve.localizer::setPoses, // A function that resets the current robot pose to the provided Pose2d
                swerve::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                swerve, // The drive subsystem
                new AutoBindings() // An empty AutoBindings object
        )).allRoutines;

        for (String name : routines.keySet()) {
            autoChooser.addRoutine(name, routines.get(name));
        }

        SmartDashboard.putData("Auto Chooser", autoChooser);
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
                        driverXbox::getRightX,
                        () -> driverXbox.leftBumper().getAsBoolean(),
                        () -> driverXbox.rightBumper().getAsBoolean()
                )
        );
    }

    private void configureBindings() {

        driverXbox.a().whileTrue(swerve.runOnce(swerve.localizer::forceUpdateQuestNavPose));

        // toggle between robot choosing quest nav pose and pose estimation with cameras
        driverXbox.b().onTrue(swerve.runOnce(swerve.localizer::toggleLocalizationStrategy));

        driverXbox.x().onTrue(swerve.runOnce(swerve.localizer::recalibrateMegaTag));

        // reset the field-centric heading on y press
        driverXbox.y().onTrue(swerve.resetGyro());

        driverXbox.povUp().whileTrue(swerve.moveToNote());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        sysID.configureBindings(driverXbox);
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(3, 3)
                .setReversed(Constants.ALLIANCE_SUPPLIER.get() == DriverStation.Alliance.Red)
                .setKinematics(swerve.getKinematics());

        Trajectory test = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.5, 5.5, Rotation2d.fromDegrees(180.0)),
                new ArrayList<>(),
                new Pose2d(7.5, 7.5, Rotation2d.fromDegrees(180.0)),
                config
        );

        SwerveControllerCommand followTestTraj = new SwerveControllerCommand(
                test,
                swerve.localizer::getStrategyPose,
                swerve.getKinematics(),
                new PIDController(Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P, 0, 0),
                new PIDController(Constants.SwerveConstants.PATH_TRANSLATION_CONTROLLER_P, 0, 0),
                new ProfiledPIDController(
                        Constants.SwerveConstants.PATH_ROTATION_CONTROLLER_P,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(
                                Constants.MAX_ANGULAR_VEL,
                                Constants.MAX_ANGULAR_VEL
                        )
                ),
                states -> swerve.setControl(
                        new SwerveRequest.ApplyFieldSpeeds().withSpeeds(swerve.getKinematics().toChassisSpeeds(states))
                ),
                swerve
        );

//        return autoChooser.selectedCommandScheduler();
        return Commands.sequence(
                Commands.runOnce(() -> swerve.localizer.setPoses(test.getInitialPose())),
                followTestTraj,
                Commands.runOnce(() -> swerve.driveFieldCentric(
                        () -> 0.0,
                        () -> 0.0,
                        () -> 0.0,
                        () -> false,
                        () -> false
                ))
        );
    }
}
