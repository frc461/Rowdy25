package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.Swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class SysID {
    private final Swerve swerve;
    private final SysIdRoutine swerveRoutine;

    public SysID(Swerve swerve) {
        this.swerve = swerve;
        swerveRoutine = configureSwerveRoutine();
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #swerveRoutine}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command swerveQuasistatic(SysIdRoutine.Direction direction) {
        return swerveRoutine.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #swerveRoutine}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command swerveDynamic(SysIdRoutine.Direction direction) {
        return swerveRoutine.dynamic(direction);
    }

    public SysIdRoutine configureSwerveRoutine() {
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
                        output -> this.swerve.setControl(
                                new SwerveRequest.SysIdSwerveTranslation().withVolts(output)
                        ),
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
                        volts -> this.swerve.setControl(
                                new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)
                        ),
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
                            this.swerve.setControl(
                                    new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts))
                            );
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        this.swerve
                )
        );

        /* The SysId routine to test */
        return sysIDTranslationRoutine;
    }

    public void configureBindings(CommandXboxController controller) {
        controller.back().and(controller.y()).whileTrue(swerveDynamic(SysIdRoutine.Direction.kForward));
        controller.back().and(controller.x()).whileTrue(swerveDynamic(SysIdRoutine.Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(swerveQuasistatic(SysIdRoutine.Direction.kForward));
        controller.start().and(controller.x()).whileTrue(swerveQuasistatic(SysIdRoutine.Direction.kReverse));
    }
}
