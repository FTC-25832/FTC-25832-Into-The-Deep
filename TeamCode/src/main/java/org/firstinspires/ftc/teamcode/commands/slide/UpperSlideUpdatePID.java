package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

public class UpperSlideUpdatePID extends CommandBase {
    private final UpperSlide upSlide;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private static final double PID_UPDATE_INTERVAL = 0.02; // 50Hz instead of loop frequency

    // Cache position reading to avoid multiple hardware calls
    private double cachedPosition = 0;
    private final ElapsedTime positionCacheTimer = new ElapsedTime();
    private static final double POSITION_CACHE_INTERVAL = 0.01; // Cache for 10ms

    public UpperSlideUpdatePID(UpperSlide upSlide) {
        this.upSlide = upSlide;
        addRequirement(upSlide);
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Update cached position reading less frequently
        if (positionCacheTimer.seconds() >= POSITION_CACHE_INTERVAL) {
            cachedPosition = upSlide.getCurrentPosition();
            positionCacheTimer.reset();
        }

        // Run PID at reduced frequency
        if (pidTimer.seconds() >= PID_UPDATE_INTERVAL) {
            upSlide.updatePID();
            pidTimer.reset();
        }

        // Telemetry uses cached values
        packet.put("UpperSlide/target", upSlide.pidfController.destination);
        packet.put("UpperSlide/current", cachedPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            upSlide.stop();
        }
    }
}
