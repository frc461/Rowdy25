package io.github.frc461.rowdy25.util;

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

import edu.wpi.first.wpilibj.DriverStation;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * Utility class for retrieving the MAC address of a connected device.
 *
 * @author Eugene Zhang, <a href="https://github.com/e500">GitHub</a>
 *
 */
public final class MacAddress {
    /**
     * Gets the MAC address of the first network interface found. Usually would be a RoboRIO when deployed on a robot.
     *
     * @return The MAC address as a String in the format "XX-XX-XX-XX-XX-XX". If no MAC address is found, returns an empty string.
     */
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
