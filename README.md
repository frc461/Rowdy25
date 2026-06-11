# FRC 461 Rowdy25 Robot Software

This repository contains the 2025 robot software for Team 461.

## Notes (Contributions, Structure, etc.)

1. Development workflows generally follow experimental feature branches and PRs, exploring various implementations and ideas. Explore different branches for your interest.

2. If deploying Robot/Auto code to hardware, be cautious deploying branches other than the `main` branch, as experimental branches may be unstable.

3. Our drivetrain is made up of 4 SDS MK4-L? swerve drives with Kraken X60 motors for rotation and driving, with CTRE CanCoders to measure wheel rotation.

4. The robot uses a command-based structure; see [WPILib command-based]((https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)) documentation for background.

<hr>

## DOCS

For in-depth documentation about this codebase (deployment, developer guides, and subsystem overviews), check out our [`doc`](doc) directory.
