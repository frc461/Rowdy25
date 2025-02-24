package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

public class SendableTelemetryUtil {
    public static synchronized void putData(String key, Sendable data, NetworkTable table) {
        NetworkTable dataTable = table.getSubTable(key);
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(data, builder);
        builder.startListeners();
        dataTable.getEntry(".name").setString(key);
    }
}
