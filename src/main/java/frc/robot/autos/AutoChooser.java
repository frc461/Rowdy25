package frc.robot.autos;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.routines.AutoEventLooper;
import frc.robot.constants.Constants;
import frc.robot.util.FieldUtil;
import frc.robot.util.MultipleChooser;

public final class AutoChooser {
    // TODO: CONFIGURE THESE VALUES, ADD ANOTHER CHOOSER
    public enum StartPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum ScoringPositions {
        A1, A2, A3, A4,
        B1, B2, B3, B4,
        C1, C2, C3, C4,
        D1, D2, D3, D4,
        E1, E2, E3, E4,
        F1, F2, F3, F4,
        G1, G2, G3, G4,
        H1, H2, H3, H4,
        I1, I2, I3, I4,
        J1, J2, J3, J4,
        K1, K2, K3, K4,
        L1, L2, L3, L4
    }

    public StartPosition startPosition;
    public List<ScoringPositions> scoringLocations;

    private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
    private final MultipleChooser<ScoringPositions> scoringLocationsChooser = new MultipleChooser<>();

    public AutoChooser() {
        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);

        for (ScoringPositions position : ScoringPositions.values()) {
            scoringLocationsChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Scoring Locations", scoringLocationsChooser);
    }


    public Command getFinalAutoCommand(Supplier<Pose2d> poseSupplier) {
        startPosition = startPositionChooser.getSelected();
        scoringLocations = scoringLocationsChooser.getSelected();

        return PathManager.generateAutoEventLooper(startPosition, scoringLocations).cmd();
    }
}
