package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * Optimized PID update command with reduced frequency for better loop times
 */
public class LowerSlideUpdatePID extends CommandBase {
    private final LowerSlide lowSlide;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private static final double PID_UPDATE_INTERVAL = 0.02; // 50Hz instead of loop frequency

    // Cache position reading to avoid multiple hardware calls
    private double cachedPosition = 0;
    private final ElapsedTime positionCacheTimer = new ElapsedTime();
    private static final double POSITION_CACHE_INTERVAL = 0.01; // Cache for 10ms

    public LowerSlideUpdatePID(LowerSlide lowSlide) {
        this.lowSlide = lowSlide;
        addRequirement(lowSlide);
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Update cached position reading less frequently
        if (positionCacheTimer.seconds() >= POSITION_CACHE_INTERVAL) {
            cachedPosition = lowSlide.getCurrentPosition();
            positionCacheTimer.reset();
        }

        // Run PID at reduced frequency
        if (pidTimer.seconds() >= PID_UPDATE_INTERVAL) {
            lowSlide.updatePID();
            pidTimer.reset();
        }

        // Telemetry uses cached values
        packet.put("lowerslide/target", lowSlide.pidfController.destination);
        packet.put("lowerslide/current", cachedPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            lowSlide.stop();
        }
    }
}
