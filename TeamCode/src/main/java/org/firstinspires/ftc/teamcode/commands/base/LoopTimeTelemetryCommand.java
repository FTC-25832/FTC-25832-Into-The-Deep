package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Command to display loop timing telemetry.
 * Add this to any command group or run in parallel to see loop timing.
 */
public class LoopTimeTelemetryCommand extends CommandBase {
        @Override
        public void execute(TelemetryPacket packet) {
                super.execute(packet); // This will add loop timing telemetry
                // You can add more custom telemetry here if you want
        }

        @Override
        public boolean isFinished() {
                return false; // Run continuously until interrupted
        }
}