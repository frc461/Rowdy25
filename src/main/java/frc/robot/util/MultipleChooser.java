package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class MultipleChooser implements Sendable, AutoCloseable {
    private static final String SELECTED = "selected";
    private static final String ACTIVE = "active";

    private static int instances;

    public MultipleChooser() {
        instances++;
        SendableRegistry.add(this, "MultipleChooser", instances);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}
