package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class MacAddress {
    // TODO WAIT (2/11): GET MAC ADDRESS OF ROBOTS
    public static final String TEST = "00-80-2F-18-50-1F";
    public static final String ALPHA = "00-80-2F-34-07-F0";
    public static final String ROWDY = "00-00-00-00-00-00";

    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> networkInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder macAddress = new StringBuilder();
            while (networkInterface.hasMoreElements()) {
                NetworkInterface tempInterface = networkInterface.nextElement();
                if (tempInterface != null) {
                    byte[] mac = tempInterface.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            macAddress.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return macAddress.toString();
                    } else {
                        DriverStation.reportWarning("Address not accessible", false);
                    }
                } else {
                    DriverStation.reportWarning("Network Interface for specified address not found", false);
                }
            }
        } catch (SocketException e) {
            DriverStation.reportError("Failed to load MAC Address: " + e.getMessage(), e.getStackTrace());
            throw new RuntimeException(e);
        }

        return "";
    }
}
