package frc.robot.telemetry;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class QuestNavTelemetry {
    NetworkTable oculusQuestTable = Constants.NT_INSTANCE.getTable("oculus");

    // TODO: figure out what topic miso and mosi does
    private IntegerSubscriber questMisoTopic = Constants.NT_INSTANCE.getIntegerTopic("miso").subscribe(0);
    private IntegerPublisher questMosiTopic = Constants.NT_INSTANCE.getIntegerTopic("mosi").publish();

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
        SmartDashboard.putNumber("quest-x-pos", questPosition[0]);
        SmartDashboard.putNumber("quest-y-pos", questPosition[1]);
        SmartDashboard.putNumber("quest-z-pos", questPosition[2]); // probably z pos, might be angle
    }

}
