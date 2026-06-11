# Vendor Dependencies (install before deployment)

The following dependencies are imported into the Rowdy25 project:
- **Phoenix v6** - CTRE Phoenix API for TalonFX and other motor controllers
- **RevLib 2025** - REV Robotics API for brushless motor controllers and Spark Max/Flex
- **PathPlanner** - Path generation for autonomous routines
- **PhotonVision** - Computer vision library for target detection and tracking
- **ReduxLib 2025** - Redux Robotics utilities for advanced motor control
- **DogLog** - Telemetry and logging framework for robot data collection
- **WPILib New Commands 2025** - WPILib's command-based framework

## Vendor Dependency JSON Links

These are the official vendor library JSON URLs used in `vendordeps/`:

- Phoenix v6: https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json
- RevLib 2025: https://software-metadata.revrobotics.com/REVLib-2025.json
- PathPlanner: https://pathplanner.dev/PathplannerLib.json
- PhotonVision: https://maven.photonvision.org/repository/snapshots/org/photonvision/PhotonLib-json/1.0-SNAPSHOT/PhotonLib-json-1.0-SNAPSHOT.json
- ReduxLib: https://maven.reduxrobotics.com/release/com/reduxrobotics/Redux-2025.0.1.json
- DogLog: https://3015rangerrobotics.github.io/DogLog/DogLog.json
- WPILib New Commands: Pre-installed with WPILib

## Installation Instructions

1. Open WPILib VSCode and load the Rowdy25 project folder.
2. Press `Ctrl+Shift+P` to open the command palette, then search for and select **"Manage Vendor Libraries"**.
3. Click **"Install new library (online)"** and paste each JSON URL above.
4. Verify all libraries are installed before building.

To check for updates on existing libraries, follow steps 1–2 above, then select **"Check for updates (online)"**.

If using an IDE other than WPILib VSCode, follow vendor-specific Gradle integration instructions in each library's documentation.

## Third-Party Library Documentation

- [Phoenix v6 Documentation](https://v6.docs.ctr-electronics.com/en/latest/)
  - [Phoenix v6 Java API Reference](https://api.ctr-electronics.com/phoenix6/release/java/)
- [RevLib Documentation](https://docs.revrobotics.com/rev-hardware-client/rev-hardware/motors)
  - [RevLib 2025 Java Docs](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [ReduxLib Documentation](https://wiki.reduxrobotics.com/)
- [DogLog Documentation](https://3015rangerrobotics.github.io/DogLog/)
- [WPILib Command-Based Framework](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)

