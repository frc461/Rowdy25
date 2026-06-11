package io.github.frc461.rowdy25.subsystems;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Utility class for controlling addressable LED strips.
 * <p>
 * Provides static methods to configure and set the LED state (on/off) for
 * visual robot status indication (e.g. coral presence).
 *
 * @author Aneesh Terani, <a href="https://github.com/aterani">GitHub</a>
 */
public class Lights {

    /** The addressable LED instance on PWM port 2. */
    private static final AddressableLED lights = new AddressableLED(2);

    /** The LED buffer holding color data for 12 LEDs. */
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(12);

    /**
     * Configures the LED strip length to match the buffer size.
     * Must be called once before using {@link #setLights(boolean)}.
     */
    public static void configureLights() {
        lights.setLength(buffer.getLength());
    }

    /**
     * Sets all LEDs on or off.
     * <p>
     * When on, all LEDs are set to orange. When off, all LEDs are turned off.
     * This starts continuous LED output.
     *
     * @param on If true, turns all LEDs orange; if false, turns all LEDs off.
     */
    public static void setLights(boolean on) {
        if (on) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, Color.kOrange);
            }
        } else {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 0, 0, 0);
            }
        }

        lights.setData(buffer);
        lights.start();
    }
}