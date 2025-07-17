package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;

/**
 * executing the grab sequence on the lower slide
 */
public class LowerSlideUpdatePID extends CommandBase {
    private final LowerSlide lowSlide;

    public LowerSlideUpdatePID(LowerSlide lowSlide) {
        this.lowSlide = lowSlide;
        addRequirement(lowSlide);
    }

    @Override
    public void execute(TelemetryPacket packet) {
        lowSlide.updatePID();
        packet.put("lowerslide/target", lowSlide.pidfController.destination);
        packet.put("lowerslide/current", lowSlide.getCurrentPosition());
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
