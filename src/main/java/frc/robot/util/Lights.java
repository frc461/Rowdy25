package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class Lights { 

    private static AddressableLED lights = new AddressableLED(2);
    private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(12);

    public static void configureLights() {
        lights.setLength(buffer.getLength());
    }

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
