# RobotIdentity

[`RobotIdentity`](../../src/main/java/io/github/frc461/rowdy25/constants/RobotIdentity.java) implements the dynamic constant selection used at robot startup.

## Purpose

Selects which variant of constants to use (`DefaultConstants` / `CompConstants` / `SimConstants` / `TestConstants`) based on the host's MAC address. This lets one codebase support multiple physical robots and simulation without changing source at runtime.

## How It Works

1. At startup `RobotIdentity.initializeConstants()` is called from the `Robot()` constructor.
2. The method reads the current host's MAC address via [`MacAddress.getMacAddress()`](../util/OTHER.md).
3. The address is matched against `RobotIdentity`'s enum entries; on match the corresponding variant class is loaded into the active [`Constants`](CONSTANTS.md) fields.
4. If no entry matches, the codebase falls back to `DefaultConstants` (this is also the path used for the alpha bot, which intentionally has no dedicated variant file).

## Known Variants

| Identity | MAC source                                | Variant class       |
|----------|-------------------------------------------|---------------------|
| `ALPHA`  | (fallback — no MAC match)                 | `DefaultConstants`  |
| `ROWDY`  | Competition RoboRIO                       | `CompConstants`     |
| `TEST`   | Test-bench RoboRIO                        | `TestConstants`     |
| `SIM`    | Simulation host                           | `SimConstants`      |

## Adding a New Robot

1. Add a new constants variant under `src/main/java/io/github/frc461/rowdy25/constants/variants/` (copy an existing one and edit values).
2. Add a new identity entry in `RobotIdentity` mapping the new RoboRIO MAC address to that variant.
3. Build and deploy; `RobotIdentity` will automatically select the new constants at runtime.

## Notes

- Variant files are source-controlled; prefer creating a new variant class instead of altering an existing competition variant.
- Keep sensitive network or environment information out of constants — variants should only hold configuration and calibration values.
