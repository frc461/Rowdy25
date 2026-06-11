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

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import io.github.frc461.rowdy25.constants.Constants;

/**
 * Telemetry publisher for the swerve drivetrain subsystem.
 * <p>
 * Publishes drivetrain state (pose, speeds, module states, current, stuck status)
 * to NetworkTables, SmartDashboard Mechanism2d visualizations, and DogLog/SignalLogger.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 * @author Geeson Wan, <a href="https://github.com/gerseneck">GitHub</a>
 */
public class SwerveTelemetry {
    /** The swerve subsystem to read state from. */
    private final Swerve swerve;

    /**
     * Constructs a SwerveTelemetry instance and starts the CTRE SignalLogger.
     *
     * @param swerve The swerve drivetrain subsystem.
     */
    public SwerveTelemetry(Swerve swerve) {
        this.swerve = swerve;
        SignalLogger.start();
    }

    /* What to publish over networktables for telemetry */

    /** NetworkTable for swerve drivetrain state. */
    private final NetworkTable driveStateTable = Constants.NT_INSTANCE.getTable("DriveState");

    /** Publisher for the robot pose as a Pose2d struct. */
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    /** Publisher for the chassis speeds as a struct. */
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

    /** Publisher for the current module states as a struct array. */
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();

    /** Publisher for the target module states as a struct array. */
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();

    /** Publisher for the module positions as a struct array. */
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

    /** Publisher for the CANcoder absolute encoder positions. */
    private final DoubleArrayPublisher cancoderAngles = driveStateTable.getDoubleArrayTopic("Module Cancoder positions").publish();

    /** Publisher for the odometry timestamp. */
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();

    /** Publisher for the odometry frequency in Hz. */
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /** Publisher for the current drive mode name. */
    private final StringPublisher currentDriveMode = driveStateTable.getStringTopic("Current Drive Mode").publish();

    /** Publisher for the current draw of each drive motor. */
    private final DoubleArrayPublisher currentCurrent = driveStateTable.getDoubleArrayTopic("Current Amp Currents").publish();

    /** Publisher for whether the drivetrain is stuck. */
    private final BooleanPublisher isStuck = driveStateTable.getBooleanTopic("Drivetrain is stuck").publish();

    /* Mechanisms to represent the swerve module states */

    /** Mechanism2d visualizations for each of the 4 swerve modules. */
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };

    /** Ligaments representing the velocity vector of each module. */
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };

    /** Ligaments representing the direction of each module. */
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /** Reusable array for logging the robot pose (x, y, rotation). */
    private final double[] poseArray = new double[3];

    /** Reusable array for logging module states (angle, speed × 4 modules). */
    private final double[] moduleStatesArray = new double[8];

    /** Reusable array for logging module targets (angle, speed × 4 modules). */
    private final double[] moduleTargetsArray = new double[8];

    /**
     * Publishes all swerve drivetrain telemetry values to NetworkTables,
     * SmartDashboard, DogLog, and SignalLogger.
     * <p>
     * Updates the Field2d pose, module state Mechanism2d visualizations,
     * current draw, stuck status, and CANcoder positions.
     */
    public void publishValues() {
        SwerveDriveState state = swerve.getState();
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
        currentDriveMode.set(swerve.getCurrentMode().name());

        double[] currents = new double[4];
        double[] positions = new double[4];

        /* Also write to log file */
        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            moduleStatesArray[i * 2] = state.ModuleStates[i].angle.getRadians();
            moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            moduleTargetsArray[i * 2] = state.ModuleTargets[i].angle.getRadians();
            moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
            currents[i] = swerve.getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble();
            positions[i] = swerve.getModule(i).getEncoder().getPosition().getValueAsDouble();
        }

        currentCurrent.set(currents); // sets the current current to currents
        isStuck.set(swerve.isStuck());
        cancoderAngles.set(positions);

        SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * Constants.MAX_VEL));

            SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
        }

        logValues(state);
    }

    /**
     * Logs swerve drivetrain state values to DogLog.
     *
     * @param state The current swerve drive state.
     */
    private void logValues(SwerveDriveState state) {
        DogLog.log("OdometryPose", state.Pose);
        DogLog.log("ChassisSpeeds", state.Speeds);
        DogLog.log("ModuleStates", state.ModuleStates);
        DogLog.log("ModuleTargets", state.ModuleTargets);
        DogLog.log("ModulePositions", state.ModulePositions);
        DogLog.log("OdometryPeriod", state.OdometryPeriod);
    }
}