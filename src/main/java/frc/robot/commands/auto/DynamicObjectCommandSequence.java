package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.PathManager;
import frc.robot.commands.DriveToObjectCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class DynamicObjectCommandSequence extends SequentialCommandGroup {

    public DynamicObjectCommandSequence(Swerve swerve, SwerveRequest.FieldCentric fieldCentric, SwerveRequest.RobotCentric robotCentric) {
        addCommands(
                new SearchForObjectCommand(swerve, fieldCentric),
                new DriveToObjectCommand(swerve, robotCentric),
                PathManager.pathFindToNearestShootingLocation(swerve.localizer::getStrategyPose)
        );
    }

}
