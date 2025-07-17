package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

/**
 * The command scheduler manages the execution of commands and coordinates
 * subsystem requirements.
 */
public class CommandScheduler {
    private static CommandScheduler instance;

    // Subsystems and their registered commands
    private final Set<SubsystemBase> subsystems = new HashSet<>();
    private final Map<SubsystemBase, Command> requirements = new HashMap<>();
    private final Set<Command> scheduledCommands = new HashSet<>();

    // Command execution tracking
    private final Map<Command, Long> commandStartTimes = new HashMap<>();
    private boolean running = true;

    private CommandScheduler() {
    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    /**
     * Register a subsystem with the scheduler.
     *
     * @param subsystem Subsystem to register
     */
    public void registerSubsystem(SubsystemBase subsystem) {
        subsystems.add(subsystem);
        subsystem.register();
    }

    /**
     * Schedule a command to run.
     *
     * @param command Command to schedule
     */
    public void schedule(Command command) {
        if (command == null) {
            return;
        }

        // Check requirements and cancel any conflicting commands
        for (SubsystemBase requirement : command.getRequirements()) {
            Command currentCommand = requirements.get(requirement);
            if (currentCommand != null && currentCommand != command) {
                cancel(currentCommand);
            }
            requirements.put(requirement, command);
            requirement.setCurrentCommand(command);
        }

        if (scheduledCommands.add(command)) {
            command.initialize();
            commandStartTimes.put(command, System.currentTimeMillis());
        }
    }

    /**
     * Cancel a running command.
     *
     * @param command Command to cancel
     */
    public void cancel(Command command) {
        if (command == null || !scheduledCommands.contains(command)) {
            return;
        }

        command.end(true);
        scheduledCommands.remove(command);
        commandStartTimes.remove(command);

        for (SubsystemBase requirement : command.getRequirements()) {
            requirements.remove(requirement);
            requirement.setCurrentCommand(null);
        }
    }

    /**
     * Run one iteration of the scheduler.
     *
     * @param packet Telemetry packet for logging
     */
    public void run(TelemetryPacket packet) {
        if (!running) {
            return;
        }

        // Run subsystem periodic methods
        for (SubsystemBase subsystem : subsystems) {
            subsystem.periodic(packet);

            // Schedule default commands if needed
            Command defaultCommand = subsystem.getDefaultCommand();
            if (defaultCommand != null && !requirements.containsKey(subsystem)) {
                schedule(defaultCommand);
            }
        }

        // Execute scheduled commands
        Iterator<Command> commandIterator = scheduledCommands.iterator();
        while (commandIterator.hasNext()) {
            Command command = commandIterator.next();

            // Check command timeout
            long timeout = command.getTimeout();
            if (timeout > 0) {
                long elapsed = System.currentTimeMillis() - commandStartTimes.get(command);
                if (elapsed >= timeout) {
                    command.end(true);
                    commandIterator.remove();
                    commandStartTimes.remove(command);
                    command.getRequirements().forEach(requirement -> {
                        requirements.remove(requirement);
                        requirement.setCurrentCommand(null);
                    });
                    continue;
                }
            }

            command.execute(packet);

            if (command.isFinished()) {
                command.end(false);
                commandIterator.remove();
                commandStartTimes.remove(command);
                command.getRequirements().forEach(requirement -> {
                    requirements.remove(requirement);
                    requirement.setCurrentCommand(null);
                });
            }
        }

        // Add telemetry
        packet.put("CommandScheduler/numSubsystems", subsystems.size());
        packet.put("CommandScheduler/numScheduledCommands", scheduledCommands.size());
    }

    /**
     * Cancel all running commands.
     */
    public void cancelAll() {
        // Create a copy of the set to avoid concurrent modification
        ArrayList<Command> commandsToCancel = new ArrayList<>(scheduledCommands);
        for (Command command : commandsToCancel) {
            cancel(command);
        }
    }

    /**
     * Enable or disable the scheduler.
     *
     * @param enabled Whether to enable the scheduler
     */
    public void setEnabled(boolean enabled) {
        running = enabled;
        if (!enabled) {
            cancelAll();
        }
    }

    /**
     * Clear all registered subsystems and commands.
     */
    public void reset() {
        cancelAll();
        subsystems.clear();
        requirements.clear();
        scheduledCommands.clear();
        commandStartTimes.clear();
    }
}
