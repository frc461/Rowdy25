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

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * A class that allows multiple selections from a list of options, typically used in a dashboard interface.
 *
 * @author Eugene Zhang, <a href="https://github.com/e500">GitHub</a>
 * @param <V> The type of values associated with the options.
 *
 */
public final class MultipleChooser<V> implements Sendable, AutoCloseable {
    /** The key for the default selection property. */
    private static final String DEFAULT = "default";
    /** The key for the selected option(s) property. */
    private static final String SELECTED = "selected";
    /** The key for the active option(s) property. */
    private static final String ACTIVE = "active";
    /** The key for the option(s) property. */
    private static final String OPTIONS = "options";
    /** The key for the instance property. */
    private static final String INSTANCE = ".instance";

    /** The map of option names to their corresponding values. */
    private final Map<String, V> options = new LinkedHashMap<>();

    /** The unique instance identifier for this MultipleChooser. */
    private final int instance;
    /** The list of default selections. */
    private final List<String> defaultSelection = new ArrayList<>();
    /** The list of previous selections to detect changes. */
    private final List<String> previousSelection = new ArrayList<>();
    /** The listener to be called when the selection changes. */
    private Consumer<List<V>> listener;
    /** Manager to keep track of the number of instances created. */
    private static final AtomicInteger instances = new AtomicInteger();

    /** Constructs a new MultipleChooser instance and registers it with the SendableRegistry. */
    public MultipleChooser() {
        instance = instances.getAndIncrement();
        SendableRegistry.add(this, "MultipleChooser", instance);
    }

    /** Closes the MultipleChooser and removes it from the SendableRegistry. */
    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Adds an option to the MultipleChooser.
     *
     * @param name  The name of the option.
     * @param value The value associated with the option.
     *
     */
    public void addOption(String name, V value) {
        options.put(name, value);
    }

    /**
     * Sets the default selection for the MultipleChooser.
     *
     * @param options A map of option names to their corresponding values to be set as default selections.
     *
     */
    public void setDefaultSelection(Map<String, V> options) {
        defaultSelection.clear();
        defaultSelection.addAll(options.keySet());
        options.keySet().forEach(option -> {
            if (!this.options.containsKey(option)) {
                this.options.put(option, options.get(option));
            }
        });
    }

    /**
     * Retrieves the currently selected values.
     *
     * @return A list of selected values. If no selections are made, returns the default selections.
     *
     */
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

    /**
     * Registers a listener to be called when the selection changes.
     *
     * @param listener A Consumer that accepts a list of selected values.
     *
     */
    public void onChange(Consumer<List<V>> listener) {
        lock.lock();
        this.listener = listener;
        lock.unlock();
    }

    /** The current selection of option names. */
    private final List<String> selection = new ArrayList<>();
    /** Lock to ensure thread-safe access to selection and listener. */
    private final ReentrantLock lock = new ReentrantLock();

    /**
     * Initializes the Sendable properties for the MultipleChooser.
     *
     * @param builder The SendableBuilder used to define the properties.
     *
     */
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
                    List<V> choice;
                    Consumer<List<V>> listener;
                    lock.lock();
                    try {
                        selection.clear();
                        selection.addAll(Arrays.stream(value.split(", ")).distinct().toList());
                        if (!selection.equals(previousSelection) && this.listener != null) {
                            choice = selection.stream().map(options::get).toList();
                            listener = this.listener;
                        } else {
                            choice = null;
                            listener = null;
                        }
                        previousSelection.clear();
                        previousSelection.addAll(selection);
                    } finally {
                        lock.unlock();
                    }
                    if (listener != null) {
                        listener.accept(choice);
                    }
                }
        );
    }
}
