# Lights Subsystem

[`Lights`](../../src/main/java/io/github/frc461/rowdy25/subsystems/Lights.java) is a thin static wrapper around a WPILib `AddressableLED` on **PWM port 2** with a 12-pixel buffer. Its entire API is two methods:

```java
public static void configureLights() { lights.setLength(buffer.getLength()); }

public static void setLights(boolean on) {
    if (on) for (int i = 0; i < buffer.getLength(); i++) buffer.setLED(i, Color.kOrange);
    else    for (int i = 0; i < buffer.getLength(); i++) buffer.setRGB(i, 0, 0, 0);
    lights.setData(buffer);
    lights.start();
}
```

## Call Sites

- `RobotStates()` calls `Lights.configureLights()` exactly once, to push the buffer length into the WPILib `AddressableLED` so it knows how many pixels to clock out.
- `Intake.periodic()` calls `Lights.setLights(hasCoral() || hasAlgae())` every tick — i.e., the LED strip glows orange whenever the intake confirms a game piece is held, and turns off otherwise.

## Why `static`?

`Lights` is *not* a WPILib `Subsystem` and is not registered with the scheduler. Because there is exactly one strip and exactly one piece of state (the boolean), a class with two static methods is sufficient — no command-based machinery needed. The flip side: nothing else can require the LED strip, so concurrent writers would race. The current design has only one writer (`Intake.periodic()`), so this is fine.

## `lights.start()` Every Tick?

`AddressableLED.start()` is idempotent after the first call — calling it every frame is wasteful but harmless. The pattern is a defensive paste that ensures the LED PWM output remains active even if some external code (e.g., a vendor library) ever called `stop()`.

## What the Indicator Means in Practice

Orange = "intake has something it shouldn't lose." This is the visual cue for:

- The driver, when looking sideways at the chassis.
- The human player at the coral station, who sees the LED change state and knows to load another coral.
- The pit crew during practice runs, who can debug "did the intake even register that?" without watching a dashboard.

## See Also

- [`Intake`](INTAKE.md) — Sole writer of the LED state.
- [`RobotStates`](../ROBOT_STATES.md) — Calls `configureLights()` exactly once at construction time.
