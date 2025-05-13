package org.firstinspires.ftc.teamcode.subsystems.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.commands.base.Command;

/**
 * Base class for all subsystems.
 * Provides basic functionality and lifecycle methods.
 */
public abstract class SubsystemBase {
        protected String name;
        protected Command defaultCommand;
        protected Command currentCommand;

        public SubsystemBase(String name) {
                this.name = name;
        }

        /**
         * Initialize the subsystem with hardware map.
         * 
         * @param hardwareMap Robot hardware map
         */
        public abstract void initialize(HardwareMap hardwareMap);

        /**
         * Read sensor values and update subsystem state.
         * Called periodically by the command scheduler.
         * 
         * @param packet Telemetry packet for logging
         */
        public void periodic(TelemetryPacket packet) {
                packet.put(name + "/hasDefaultCommand", defaultCommand != null);
                packet.put(name + "/hasCurrentCommand", currentCommand != null);
                if (currentCommand != null) {
                        // packet.put(name + "/currentCommand", currentCommand.getName());
                        packet.put(name + "/currentCommand", currentCommand);
                }
        }

        /**
         * Set the default command for this subsystem.
         * 
         * @param command Command to run by default
         */
        public void setDefaultCommand(Command command) {
                defaultCommand = command;
        }

        /**
         * Get the default command for this subsystem.
         * 
         * @return Default command
         */
        public Command getDefaultCommand() {
                return defaultCommand;
        }

        /**
         * Get the current command running on this subsystem.
         * 
         * @return Current command
         */
        public Command getCurrentCommand() {
                return currentCommand;
        }

        /**
         * Set the current command running on this subsystem.
         * 
         * @param command Command to run
         */
        public void setCurrentCommand(Command command) {
                this.currentCommand = command;
        }

        /**
         * Get the name of this subsystem.
         * 
         * @return Subsystem name
         */
        public String getName() {
                return name;
        }

        /**
         * Register periodic callbacks for this subsystem.
         * Override this method to register any periodic callbacks needed.
         */
        public void register() {
        }

        /**
         * Stop all hardware devices and cleanup subsystem state.
         * Called when OpMode is stopping to ensure safe shutdown.
         * Must be implemented by each subsystem to handle their specific hardware.
         */
        public abstract void stop();
}
