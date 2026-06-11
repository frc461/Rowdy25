# Vendor Dependencies

Rowdy25 imports the following vendor libraries. Their JSON manifests live under [`vendordeps/`](../../vendordeps/) at the repository root and are picked up automatically by the Gradle build.

## Imported Libraries

- **Phoenix 6** — CTRE motor control API for TalonFX (Kraken X60 / Falcon 500), CANcoder, Pigeon 2, etc. Used by every subsystem.
- **REVLib 2025** — REV Robotics API for SPARK MAX / SPARK Flex brushless controllers.
- **PathPlanner** — Path generation, follow-path commands, `LocalADStar` pathfinding, and named-command event markers for autonomous.
- **PhotonVision** — Vision library for AprilTag pose estimation and ML/color-based object detection.
- **ReduxLib 2025** — Redux Robotics utilities; provides the CANandcolor sensor used by the intake and other auxiliary sensors.
- **DogLog** — Lightweight telemetry/logging framework used throughout the codebase for state and signal logging.
- **WPILib New Commands 2025** — WPILib's command-based framework (pre-installed with WPILib; listed here for completeness).

## Vendor Manifest URLs

These are the official vendor library JSON URLs that match the manifests committed in `vendordeps/`. Use them when refreshing or re-installing libraries.

- Phoenix 6: <https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2025-latest.json>
- REVLib 2025: <https://software-metadata.revrobotics.com/REVLib-2025.json>
- PathPlanner: <https://pathplanner.dev/PathplannerLib.json>
- PhotonVision: <https://maven.photonvision.org/repository/snapshots/org/photonvision/PhotonLib-json/1.0-SNAPSHOT/PhotonLib-json-1.0-SNAPSHOT.json>
- ReduxLib: <https://maven.reduxrobotics.com/release/com/reduxrobotics/Redux-2025.0.1.json>
- DogLog: <https://3015rangerrobotics.github.io/DogLog/DogLog.json>
- WPILib New Commands: pre-installed with WPILib

## Installation Instructions

1. Open the Rowdy25 project folder in WPILib VSCode.
2. Press <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd> to open the command palette and run **"WPILib: Manage Vendor Libraries"**.
3. Choose **"Install new library (online)"** and paste each JSON URL above.
4. Verify all libraries appear under `vendordeps/` and that the project builds (`./gradlew build`).

To check for updates on existing libraries, follow steps 1–2 above and then select **"Check for updates (online)"**.

If you're using an IDE other than WPILib VSCode, follow each vendor's Gradle integration instructions — the JSON manifests in `vendordeps/` are still the authoritative source.

## Official Library Documentation

- [Phoenix 6 documentation](https://v6.docs.ctr-electronics.com/en/latest/)
  - [Phoenix 6 Java API reference](https://api.ctr-electronics.com/phoenix6/release/java/)
- [REVLib documentation](https://docs.revrobotics.com/rev-hardware-client/rev-hardware/motors)
  - [REVLib 2025 Java docs](https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html)
- [PathPlanner documentation](https://pathplanner.dev/)
- [PhotonVision documentation](https://docs.photonvision.org/)
- [ReduxLib documentation](https://wiki.reduxrobotics.com/)
- [DogLog documentation](https://3015rangerrobotics.github.io/DogLog/)
- [WPILib command-based framework](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html)
