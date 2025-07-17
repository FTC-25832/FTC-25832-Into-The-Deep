package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.*;

/**
 * PERFORMANCE OPTIMIZED: The command scheduler manages the execution of
 * commands and coordinates
 * subsystem requirements using optimized data structures for hot path
 * performance.
 */
public class CommandScheduler {
        private static CommandScheduler instance;

        // PERFORMANCE OPTIMIZATION: Use ArrayList instead of HashSet for better
        // iteration performance
        private final List<SubsystemBase> subsystems = new ArrayList<>();
        private final Map<SubsystemBase, Command> requirements = new HashMap<>();
        // PERFORMANCE OPTIMIZATION: Use ArrayList for commands to avoid iterator
        // overhead
        private final List<Command> scheduledCommands = new ArrayList<>();

        // Command execution tracking - keep HashMap for O(1) lookups
        private final Map<Command, Long> commandStartTimes = new HashMap<>();
        private boolean running = true;

        // PERFORMANCE OPTIMIZATION: Pre-allocated collections to avoid garbage
        // collection
        private final List<Command> commandsToRemove = new ArrayList<>();
        private final List<SubsystemBase> requirementsToRemove = new ArrayList<>();

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
         * PERFORMANCE OPTIMIZED: Run one iteration of the scheduler using optimized
         * operations.
         * 
         * @param packet Telemetry packet for logging
         */
        public void run(TelemetryPacket packet) {
                if (!running) {
                        return;
                }

                // Cache current time once per loop iteration
                long currentTime = System.currentTimeMillis();

                // PERFORMANCE OPTIMIZATION: Use indexed for-loop instead of enhanced for-loop
                // for better performance
                int subsystemCount = subsystems.size();
                for (int i = 0; i < subsystemCount; i++) {
                        SubsystemBase subsystem = subsystems.get(i);
                        subsystem.periodic(packet);

                        // Schedule default commands if needed
                        Command defaultCommand = subsystem.getDefaultCommand();
                        if (defaultCommand != null && !requirements.containsKey(subsystem)) {
                                schedule(defaultCommand);
                        }
                }

                // PERFORMANCE OPTIMIZATION: Clear pre-allocated list and collect commands to
                // remove
                commandsToRemove.clear();

                // PERFORMANCE OPTIMIZATION: Use indexed for-loop to avoid iterator overhead
                int commandCount = scheduledCommands.size();
                for (int i = 0; i < commandCount; i++) {
                        Command command = scheduledCommands.get(i);

                        // Check command timeout using cached time
                        long timeout = command.getTimeout();
                        if (timeout > 0) {
                                Long startTime = commandStartTimes.get(command);
                                if (startTime != null) {
                                        long elapsed = currentTime - startTime;
                                        if (elapsed >= timeout) {
                                                command.end(true);
                                                commandsToRemove.add(command);
                                                continue;
                                        }
                                }
                        }

                        command.execute(packet);

                        if (command.isFinished()) {
                                command.end(false);
                                commandsToRemove.add(command);
                        }
                }

                // PERFORMANCE OPTIMIZATION: Remove finished commands in batch using
                // pre-allocated collections
                if (!commandsToRemove.isEmpty()) {
                        for (Command command : commandsToRemove) {
                                scheduledCommands.remove(command);
                                commandStartTimes.remove(command);

                                // PERFORMANCE OPTIMIZATION: Collect requirements to remove to avoid iterator
                                requirementsToRemove.clear();
                                for (SubsystemBase requirement : command.getRequirements()) {
                                        requirementsToRemove.add(requirement);
                                }

                                // Remove requirements in batch
                                for (SubsystemBase requirement : requirementsToRemove) {
                                        requirements.remove(requirement);
                                        requirement.setCurrentCommand(null);
                                }
                        }
                }

                // Add telemetry (only if needed to reduce overhead)
                if (packet != null) {
                        packet.put("CommandScheduler/numSubsystems", subsystemCount);
                        packet.put("CommandScheduler/numScheduledCommands", scheduledCommands.size());
                }
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
