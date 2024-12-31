package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.util.VisionUtil;

import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FollowPathDynamicCommand extends FollowPathCommand {
    private final Timer timer = new Timer();
    private final PathPlannerPath originalPath;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlipPath;
    private final BooleanSupplier noteIsThere = VisionUtil.Photon.Color::hasTargets;
    private final List<OneShotTriggerEvent> allInstantEvents = new ArrayList<>();
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private boolean end;

    public FollowPathDynamicCommand(PathPlannerPath path, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier, BiConsumer<ChassisSpeeds, DriveFeedforwards> output, PathFollowingController controller, RobotConfig robotConfig, BooleanSupplier shouldFlipPath, Swerve swerve) {
        super(path, poseSupplier, speedsSupplier, output, controller, robotConfig, shouldFlipPath, swerve);
        this.originalPath = path;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.robotConfig = robotConfig;
        this.shouldFlipPath = shouldFlipPath;
        end = false;

        this.path = this.originalPath;
        Optional<PathPlannerTrajectory> idealTrajectory = this.path.getIdealTrajectory(this.robotConfig);
        idealTrajectory.ifPresent((traj) -> this.trajectory = traj);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (this.shouldFlipPath.getAsBoolean() && !this.originalPath.preventFlipping) {
            this.path = this.originalPath.flipPath();
        } else {
            this.path = this.originalPath;
        }

        Pose2d currentPose = this.poseSupplier.get();
        ChassisSpeeds currentSpeeds = this.speedsSupplier.get();
        double linearVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        if (this.path.getIdealStartingState() != null) {
            boolean idealVelocity = Math.abs(linearVel - this.path.getIdealStartingState().velocityMPS()) <= (double)0.25F;
            boolean idealRotation = !this.robotConfig.isHolonomic || Math.abs(currentPose.getRotation().minus(this.path.getIdealStartingState().rotation()).getDegrees()) <= (double)30.0F;
            if (idealVelocity && idealRotation) {
                this.trajectory = this.path.getIdealTrajectory(this.robotConfig).orElseThrow();
            } else {
                this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.robotConfig);
            }
        } else {
            this.trajectory = this.path.generateTrajectory(currentSpeeds, currentPose.getRotation(), this.robotConfig);
        }

        List<Event> allEvents = trajectory.getEvents();
        for (Event event : allEvents) {
            if (event instanceof OneShotTriggerEvent && ((OneShotTriggerEvent) event).getEventName().equals(Constants.AutoConstants.NOTE_CHECK_MARKER)) {
                allInstantEvents.add((OneShotTriggerEvent) event);
            }
        }

        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        super.execute();

        double currentTime = this.timer.get();
        if (!allInstantEvents.isEmpty()) {
            if (allInstantEvents.get(0).getTimestampSeconds() <= currentTime) {
                allInstantEvents.remove(0);
                if (!noteIsThere.getAsBoolean()) {
                    end = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || end;
    }
}
