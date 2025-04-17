package frc.robot.subsystems.drivetrain;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;

public class SwerveTelemetry {
    private final Swerve swerve;

    /**
     * Construct a telemetry object, with the specified swerve subsystem
     *
     * @param swerve Instance of the swerve subsystem
     */
    public SwerveTelemetry(Swerve swerve) {
        this.swerve = swerve;
        SignalLogger.start();
    }

    /* What to publish over networktables for telemetry */

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = Constants.NT_INSTANCE.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoubleArrayPublisher cancoderAngles = driveStateTable.getDoubleArrayTopic("Module Cancoder positions").publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();
    private final StringPublisher currentDriveMode = driveStateTable.getStringTopic("Current Drive Mode").publish();
    private final DoubleArrayPublisher currentCurrent = driveStateTable.getDoubleArrayTopic("Current Amp Currents").publish();
    private final BooleanPublisher isStuck = driveStateTable.getBooleanTopic("Drivetrain is stuck").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
        moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
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

    private final double[] poseArray = new double[3];
    private final double[] moduleStatesArray = new double[8];
    private final double[] moduleTargetsArray = new double[8];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
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

    private void logValues(SwerveDriveState state) {
        DogLog.log("OdometryPose", state.Pose);
        DogLog.log("ChassisSpeeds", state.Speeds);
        DogLog.log("ModuleStates", state.ModuleStates);
        DogLog.log("ModuleTargets", state.ModuleTargets);
        DogLog.log("ModulePositions", state.ModulePositions);
        DogLog.log("OdometryPeriod", state.OdometryPeriod);
    }
}
