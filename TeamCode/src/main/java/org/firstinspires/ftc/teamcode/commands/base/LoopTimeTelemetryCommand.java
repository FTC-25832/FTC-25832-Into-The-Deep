package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Command to display loop timing telemetry.
 * Add this to any command group or run in parallel to see loop timing.
 */
public class LoopTimeTelemetryCommand extends CommandBase {
    private long lastLoopTime = 0;
    private long loopCount = 0;
    private double averageLoopTime = 0;

    @Override
    public void initialize() {
        lastLoopTime = System.nanoTime();
        loopCount = 0;
        averageLoopTime = 0;
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Calculate loop time
        long currentTime = System.nanoTime();
        long loopTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;

        // Update average loop time
        loopCount++;
        averageLoopTime = (averageLoopTime * (loopCount - 1) + loopTime) / loopCount;

        // Add telemetry
        packet.put("CommandBase/loopTime_ms", loopTime / 1e6);
        packet.put("CommandBase/avgLoopTime_ms", averageLoopTime / 1e6);
        packet.put("CommandBase/loopCount", loopCount);
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously until interrupted
    }

    @Override
    public void end(boolean interrupted) {

    }
}