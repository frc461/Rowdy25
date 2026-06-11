# RobotIdentity Class

`RobotIdentity.java` implements the dynamic constant selection used at robot startup.

## Purpose

Selects which variant of constants to use (Default/Comp/Sim/Test) based on the machine's MAC address. This allows a single codebase to support multiple physical robots and simulation configurations without changing source files at runtime.

## How it works

1. At startup `RobotIdentity.initializeConstants()` is called (from `Robot()` constructor).
2. The method reads the current host's MAC address using `MacAddress.getMacAddress()`.
3. The MAC address is matched against a table of known addresses; when a match is found the corresponding variant class (e.g., `CompConstants`) is loaded into the active `Constants` references.
4. If no match is found, a default variant (usually `DefaultConstants`) is selected.

## Adding a new robot

To add a new robot/variant:

1. Add a new constants variant class under `src/main/java/io/github/frc461/rowdy25/constants/variants/` (copy an existing one and edit values).
2. Add the new robot's MAC address and mapping entry inside `RobotIdentity.initializeConstants()` so the code can recognize that robot at startup.
3. Build and deploy to the new RoboRIO; `RobotIdentity` will automatically select the appropriate constants at runtime.

## Notes

- Variant files are source-controlled; prefer creating a new variant class instead of altering an existing competition variant.
- Keep sensitive network or environment information out of constants; variants should only include configuration and calibration values.

