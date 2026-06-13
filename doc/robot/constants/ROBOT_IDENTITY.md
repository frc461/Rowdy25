# RobotIdentity Class

[`RobotIdentity`](../../src/main/java/io/github/frc461/rowdy25/constants/RobotIdentity.java) is the **MAC-address dispatch** that loads the right per-robot constants at startup. It is the first thing the [`Robot`](../ROBOT.md) constructor calls; everything downstream depends on this having already rebound the static fields of `Constants`.

## The Enum

```java
public enum RobotIdentity {
    TEST ("00-80-2F-18-50-1F"),
    ALPHA("00-80-2F-34-07-F0"),
    ROWDY("00-80-2F-33-9F-37"),
    SIM  ("");
}
```

Each value carries the NI roboRIO's primary NIC MAC address. CTRE's stock roboRIOs have a fixed Ethernet MAC burned into the controller — the four values above are the team's three physical roboRIOs plus the simulated "no MAC" case.

## `getIdentity()`

```java
String mac = MacAddress.getMACAddress();
for (RobotIdentity id : values()) {
    if (id.mac.equals(mac)) return id;
}
return SIM;
```

Simple linear search. `MacAddress.getMACAddress()` enumerates `NetworkInterface.getNetworkInterfaces()` and returns the first non-virtual hardware address (see [`OTHER.md`](../util/OTHER.md)). If no match is found, the code is assumed to be running in the simulator and `SIM` is returned. This is what makes hot-reloading on a laptop "just work" — there's no manual flag to toggle.

## `initializeConstants()`

The full sequence:

1. `setDefaultConstants()` — pull *every* field from `DefaultConstants` into the corresponding `Constants` field. This is hundreds of assignments — one per tunable in the project — done explicitly because Java doesn't have a generic mechanism for "copy all static fields of class A to class B." The default values are safe placeholders intended for `ALPHA` (the development bot, which the team uses as the baseline).
2. `Constants.IDENTITY = getIdentity()` — cache the detected identity so subsystems like [`Elevator`](../subsystems/ELEVATOR.md) can branch on it (e.g., disable the limit-switch homing for the `ROWDY` bot whose switch is broken).
3. Publish the identity name to NetworkTables under `Robot/Robot Identity` for dashboard display.
4. **`switch (getIdentity())`** — invoke the variant-specific override method, which selectively re-assigns the fields that differ for this bot:
   - `ALPHA` → no overrides (defaults are correct).
   - `ROWDY` → `setCompConstants()` (the competition robot has different swerve geometry, camera mounts, pivot encoder offsets, and most gravity-feedforward gains).
   - `TEST` → `setTestConstants()` (the test bed has only a swerve drivetrain and reduced vision config).
   - `SIM` → `setSimConstants()` (mostly tweaks D-gains because the sim model has less inertia than the real chassis).

## The "two-pass" pattern

The same field appears on both `DefaultConstants` and `CompConstants` (etc.). This was a deliberate choice over inheritance:

- **Inheritance** would have meant `CompConstants extends DefaultConstants`, which is conceptually clean but breaks down because some fields need to *vary their values per robot* (e.g., a TalonFX inversion bool), and Java prohibits overriding `static` fields.
- **Two-pass assignment** means everything has a default baseline; per-robot variant methods only need to enumerate the fields that *actually differ*. The price is the long manual assignment list in each method.

If you add a new tunable to `Constants`, you must:

1. Add it to `DefaultConstants` with a sensible default value.
2. Add the assignment line in `setDefaultConstants()`.
3. If any variant needs a different value, add it to that variant class and add the corresponding assignment in the variant's setter method.

Missing step 2 is a silent NPE at runtime; missing step 3 just means the variant gets the default value.

## Variant Classes

- `DefaultConstants` — baseline (used by `ALPHA`).
- `CompConstants` — competition robot `ROWDY`. Overrides ≈ 100 fields including swerve module IDs, CANcoder magnet offsets, camera mounts (`BW_TOP_*`, `BW_BACK_*` mount transforms differ from alpha), elevator/pivot/wrist mechanical constants, and the climb-profile `EXPO_V_SLOW`.
- `TestConstants` — test bed; overrides only the swerve and the two front cameras.
- `SimConstants` — overrides only the angular controllers' D gains.

## Why this matters at runtime

Because the `Constants` fields are `public static` and *re-assigned* (not `final`), any class that captured a reference inside a static initializer **before** `RobotIdentity.initializeConstants()` ran would see the original default value, not the variant override. This is why:

- `Robot()` calls `RobotIdentity.initializeConstants()` *before* `new RobotContainer()`.
- Variant-dependent fields are accessed lazily through `Constants.X` (a static field read) rather than cached in subsystem constants at class-init time.

A handful of fields are intentionally cached (e.g., the four `SwerveModuleConstants` for the modules) because they are read once during `Swerve` construction — which happens *after* `initializeConstants()` has already overridden them.

## See Also

- [`Constants`](CONSTANTS.md) — Shape of the data being dispatched.
- [`MacAddress`](../util/OTHER.md) — The NIC enumeration helper.
- [`Robot`](../ROBOT.md) — Calls `initializeConstants()` first thing in its constructor.
- [`Elevator`](../subsystems/ELEVATOR.md) — A concrete example of branching on `Constants.IDENTITY`.
