package frc.robot.telemetry;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class QuestNavTelemetry {
    NetworkTable oculusQuestTable = Constants.NT_INSTANCE.getTable("oculus");

    private IntegerSubscriber questFrameCountTopic = oculusQuestTable.getIntegerTopic("frameCount").subscribe(0);
    private DoubleSubscriber questTimestampTopic = oculusQuestTable.getDoubleTopic("timestamp").subscribe(0.0f);
    private DoubleSubscriber questBatteryTopic = oculusQuestTable.getDoubleTopic("batteryLevel").subscribe(0.0f);
    private FloatArraySubscriber questPositionTopic = oculusQuestTable.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questQuaternionTopic = oculusQuestTable.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questEulerAnglesTopic = oculusQuestTable.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private long questFrameCount;
    private double questTimestamp;
    private double questBattery;
    private float[] questPosition;
    private float[] questQuaternion;
    private float[] questEulerAngles;

    public QuestNavTelemetry() {
        SignalLogger.start();
    }

    public void refreshValues() {
        oculusQuestTable = Constants.NT_INSTANCE.getTable("oculus");

        questFrameCount = questFrameCountTopic.get(0);
        questTimestamp = questTimestampTopic.get(0.0f);
        questBattery = questBatteryTopic.get(0.0f);
        questPosition = questPositionTopic.get(new float[] {0.0f, 0.0f, 0.0f});
        questQuaternion = questQuaternionTopic.get(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
        questEulerAngles = questEulerAnglesTopic.get(new float[] {0.0f, 0.0f, 0.0f});
    }

    public void publishValues() {
        refreshValues();

        SmartDashboard.putNumber("quest-frame-count", questFrameCount);
        SmartDashboard.putNumber("quest-timestamp", questTimestamp);
        SmartDashboard.putNumber("quest-battery-level", questBattery);

        // physics coordinate plane (y is up)
        SmartDashboard.putNumber("quest-x-pos", questPosition[0]);
        SmartDashboard.putNumber("quest-y-pos", questPosition[1]);
        SmartDashboard.putNumber("quest-z-pos", questPosition[2]);

        // SmartDashboard.putNumber("quest-quaternion-x", questQuaternion[0]);
        // SmartDashboard.putNumber("quest-quaternion-y", questQuaternion[1]);
        // SmartDashboard.putNumber("quest-quaternion-z", questQuaternion[2]);
        // SmartDashboard.putNumber("quest-quaternion-w", questQuaternion[3]);

        // euler angle y is for rotation (yaw)
        // SmartDashboard.putNumber("quest-euler-angle-x", questEulerAngles[0]);
        SmartDashboard.putNumber("quest-euler-angle-y", questEulerAngles[1]);
        // SmartDashboard.putNumber("quest-euler-angle-z", questEulerAngles[2]);
    }
}
