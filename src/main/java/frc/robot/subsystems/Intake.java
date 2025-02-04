package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Lights;

import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    private enum States {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    private final TalonFX motor;
    private final DigitalInput coralBeam;
    private final DigitalInput algaeBeam;
    private States currentState;
    private boolean stateChanged;

    public Intake() {
        motor = new TalonFX(Constants.IntakeConstants.MOTOR_ID);

        motor.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.IntakeConstants.INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        coralBeam = new DigitalInput(Constants.IntakeConstants.CORAL_BEAM_ID);
        algaeBeam = new DigitalInput(Constants.IntakeConstants.ALGAE_BEAM_ID);
        currentState = States.IDLE;
    }

    public void setIntakeSpeed(double speed) {
        motor.set(speed);
    }
 
    public boolean hasCoral() {
        return !coralBeam.get();
    }

    public boolean hasAlgae() {
        return !algaeBeam.get();
    }

    public boolean hasGamePiece() {
        return hasCoral() || hasAlgae();
    }

    public void intake() {
        currentState = States.INTAKE;
        stateChanged = true;
    }

    public void outtake() {
        currentState = States.OUTTAKE;
        stateChanged = true;
    }

    @Override
    public void periodic() {
       Lights.setLights(hasCoral() || hasAlgae());

       if (stateChanged) {
           switch (currentState) {
               case IDLE:
                   this.getCurrentCommand().cancel();
               case INTAKE:
                   setDefaultCommand(Commands.none());
               case OUTTAKE:
                   setDefaultCommand(Commands.none());
           }
           stateChanged = false;
       }
    }
    
}
