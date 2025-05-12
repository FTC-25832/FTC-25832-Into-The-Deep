package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.Command;

/**
 * Base class for one-shot servo commands
 */
public abstract class ServoCommand implements Command {
        private boolean executed = false;
        private final String telemetryKey;
        private final double targetPosition;

        public ServoCommand(String telemetryKey, double targetPosition) {
                this.telemetryKey = telemetryKey;
                this.targetPosition = targetPosition;
        }

        @Override
        public void initialize() {
                setServoPosition();
                executed = true;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
                if (!executed) {
                        initialize();
                }
                if (telemetryKey != null) {
                        packet.put(telemetryKey, targetPosition);
                }
                return !executed;
        }

        protected abstract void setServoPosition();

        protected double getTargetPosition() {
                return targetPosition;
        }
}
