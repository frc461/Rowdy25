package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.util.FieldUtil;
import frc.robot.util.MultipleChooser;

public final class AutoChooser {
    public enum StartPosition {
        FAR_LEFT(1),
        CENTER_LEFT(2),
        CENTER(3),
        CENTER_RIGHT(4),
        FAR_RIGHT(5);

        final int index;
        StartPosition(int index) {
            this.index = index;
        }
    }

    public StartPosition startPosition;
    public List<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocations;

    private final SendableChooser<StartPosition> startPositionChooser = new SendableChooser<>();
    private final MultipleChooser<Pair<FieldUtil.Reef.ScoringLocation, FieldUtil.Reef.Level>> scoringLocationsChooser = new MultipleChooser<>();

    public AutoChooser() {
        for (StartPosition position : StartPosition.values()) {
            startPositionChooser.addOption(position.name(), position);
        }
        SmartDashboard.putData("Start Position", startPositionChooser);

        for (FieldUtil.Reef.ScoringLocation location : FieldUtil.Reef.ScoringLocation.values()) {
            for (FieldUtil.Reef.Level level : FieldUtil.Reef.Level.values()) {
                scoringLocationsChooser.addOption(location.name() + level.level, new Pair<>(location, level));
            }
        }
        SmartDashboard.putData("Scoring Locations", scoringLocationsChooser);
    }


    public Command getFinalAutoCommand(RobotStates robotStates) {
        startPosition = startPositionChooser.getSelected();
        scoringLocations = scoringLocationsChooser.getSelected();

        return PathManager.generateAutoEventLooper(startPosition, scoringLocations, robotStates).cmd();
    }
}
