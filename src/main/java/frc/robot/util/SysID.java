package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class SysID {
    private final Swerve swerve;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation driveCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine sysIDRoutineToApply;

    public SysID(Swerve swerve) {
        this.swerve = swerve;

        /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
        // Use default ramp rate (1 V/s)
        // Reduce dynamic step voltage to 4 V to prevent brownout
        // Use default timeout (10 s)
        // Log state with SignalLogger class
        SysIdRoutine sysIDTranslationRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,        // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                        null,        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        output -> this.swerve.setControl(driveCharacterization.withVolts(output)),
                        null,
                        this.swerve
                )
        );

        /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
        SysIdRoutine sysIDSteerRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,        // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use dynamic voltage of 7 V
                        null,        // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        volts -> swerve.setControl(steerCharacterization.withVolts(volts)),
                        null,
                        this.swerve
                )
        );

        /*
         * SysId routine for characterizing rotation.
         * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
         * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
         */
        SysIdRoutine sysIDRotRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        /* This is in radians per second², but SysId only supports "volts per second" */
                        Volts.of(Math.PI / 6).per(Second),
                        /* This is in radians per second, but SysId only supports "volts" */
                        Volts.of(Math.PI),
                        null, // Use default timeout (10 s)
                        // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        output -> {
                            /* output is actually radians per second, but SysId only supports "volts" */
                            swerve.setControl(rotCharacterization.withRotationalRate(output.in(Volts)));
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        this.swerve
                )
        );

        /* The SysId routine to test */
        sysIDRoutineToApply = sysIDTranslationRoutine;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIDRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIDRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIDRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIDRoutineToApply.dynamic(direction);
    }

    public void configureBindings(CommandXboxController controller) {
        controller.back().and(controller.y()).whileTrue(sysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.back().and(controller.x()).whileTrue(sysIdDynamic(SysIdRoutine.Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.start().and(controller.x()).whileTrue(sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }
}
