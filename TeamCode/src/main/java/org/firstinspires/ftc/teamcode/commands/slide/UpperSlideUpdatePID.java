package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

/**
 * executing the grab sequence on the lower slide
 */
public class UpperSlideUpdatePID extends CommandBase {
    private final UpperSlide upSlide;

    public UpperSlideUpdatePID(UpperSlide upSlide) {
        this.upSlide = upSlide;
        addRequirement(upSlide);
    }

    @Override
    public void execute(TelemetryPacket packet) {
        upSlide.updatePID();
        packet.put("UpperSlide/target", upSlide.pidfController.destination);
        packet.put("UpperSlide/current", upSlide.getCurrentPosition());
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
