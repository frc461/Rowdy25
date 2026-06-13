# RobotPoses Class

[RobotPoses](../../src/main/java/io/github/frc461/rowdy25/constants/RobotPoses.java) computes the target robot `Pose2d`s for every field landmark relevant to scoring, intaking, and climbing during the Reefscape game. Where `FieldUtil` describes the *field* (where things are), `RobotPoses` describes where the *robot* should sit relative to those features.

## Landmark Groups

- **Reef Branches** — Robot poses for scoring coral at L1–L4 on every reef branch
- **Coral Station** — Pickup poses at the left and right coral stations (including obstructed and ground-intake variants)
- **Algae on Reef** — Approach poses for removing low and high algae from each reef face
- **Processor** — Robot pose for scoring algae into the processor
- **Net** — Robot pose for scoring algae into the net (with randomized X for defense)

## Alliance Awareness

All poses are defined in blue-alliance field coordinates. Consumers pass the alliance through `Constants.ALLIANCE_SUPPLIER`; PathPlanner's `FlippingUtil` is used wherever a mirrored red-alliance pose is required.

## Usage

Used by [AutoManager](../autos/AUTO_MANAGER.md) and the Swerve `pathFindTo*` helpers (which build [`PathfindToPoseAvoidingReefCommand`](../commands/DRIVE.md) instances) to compute reachable target poses, and by [FieldUtil](../util/OTHER.md) for nearest-target queries. [`Pathfinder`](../autos/PATHFINDER.md) also reads from these tables but is currently unused by production code.

## Tuning

Verify the underlying field landmark constants (`FieldUtil`, `Constants.VisionConstants`) and any offset values match the actual field layout during competition setup; small offset corrections here are often required after field calibration.
