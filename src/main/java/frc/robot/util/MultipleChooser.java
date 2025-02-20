package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

public class MultipleChooser<V> implements Sendable, AutoCloseable { // TODO SHOP: TEST THIS CLASS
    private static final String DEFAULT = "default";
    private static final String SELECTED = "selected";
    private static final String ACTIVE = "active";
    private static final String OPTIONS = "options";
    private static final String INSTANCE = ".instance";

    private final Map<String, V> options = new LinkedHashMap<>();

    private final int instance;
    private final List<String> defaultSelection = new ArrayList<>();
    private static final AtomicInteger instances = new AtomicInteger();

    public MultipleChooser() {
        instance = instances.getAndIncrement();
        SendableRegistry.add(this, "MultipleChooser", instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    public void addOption(String name, V value) {
        options.put(name, value);
    }

    public void setDefaultSelection(Map<String, V> options) {
        defaultSelection.clear();
        defaultSelection.addAll(options.keySet());
        options.keySet().forEach(option -> {
            if (!this.options.containsKey(option)) {
                this.options.put(option, options.get(option));
            }
        });
    }

    public List<V> getSelected() {
        lock.lock();
        try {
            if (selection.isEmpty()) {
                return defaultSelection.stream().map(options::get).toList();
            }
            return selection.stream().map(options::get).toList();
        } finally {
            lock.unlock();
        }
    }

    private final List<String> selection = new ArrayList<>();
    private final ReentrantLock lock = new ReentrantLock();

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Multiple Chooser");
        builder.publishConstInteger(INSTANCE, instance);
        builder.addStringArrayProperty(DEFAULT, () -> defaultSelection.toArray(new String[0]), null);
        builder.addStringArrayProperty(OPTIONS, () -> options.keySet().toArray(new String[0]), null);
        builder.addStringArrayProperty(
                ACTIVE,
                () -> {
                    lock.lock();
                    try {
                        if (selection.isEmpty()) {
                            return defaultSelection.toArray(new String[0]);
                        }
                        return selection.toArray(new String[0]);
                    } finally {
                        lock.unlock();
                    }
                },
                null
        );
        builder.addStringProperty(
                SELECTED,
                () -> {
                    lock.lock();
                    try {
                        StringBuilder total = new StringBuilder();
                        if (!selection.isEmpty()) {
                            for (String value : selection) {
                                total.append(value).append(", ");
                            }
                            return total.substring(0, total.length() - 2);
                        }
                        return "None selected";
                    } finally {
                        lock.unlock();
                    }
                },
                value -> {
                    lock.lock();
                    try {
                        selection.clear();
                        selection.addAll(Arrays.asList(value.split(", ")));
                    } finally {
                        lock.unlock();
                    }
                }
        );
    }
}
