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

    // Subscribe to the Network Tables oculus data topics
    private IntegerSubscriber questFrameCountTopic = oculusQuestTable.getIntegerTopic("frameCount").subscribe(0);
    private DoubleSubscriber questTimestampTopic = oculusQuestTable.getDoubleTopic("timestamp").subscribe(0.0f);
    private FloatArraySubscriber questPositionTopic = oculusQuestTable.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questQuaternionTopic = oculusQuestTable.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    
    private long questFrameCount;
    private double questTimestamp;
    private float[] questPosition;
    private float[] questQuaternion;

    public QuestNavTelemetry() {
        SignalLogger.start();
    }

    public void refreshValues() {
        oculusQuestTable = Constants.NT_INSTANCE.getTable("oculus");
        questFrameCount = questFrameCountTopic.get(0);
        questTimestamp = questTimestampTopic.get(0.0f);
        questPosition = questPositionTopic.get(new float[] {0.0f, 0.0f, 0.0f});
        questQuaternion = questQuaternionTopic.get(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    }

    public void publishValues() {
        refreshValues();
        SmartDashboard.putNumber("X-Pos", questPosition[0]);
        SmartDashboard.putNumber("Y-Pos", questPosition[1]);
        SmartDashboard.putNumber("quest-timestamp", questTimestamp);
    }

}
