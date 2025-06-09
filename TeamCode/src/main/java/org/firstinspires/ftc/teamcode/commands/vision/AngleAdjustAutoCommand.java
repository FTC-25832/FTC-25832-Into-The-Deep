package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * Command to perform vision-assisted adjustments using Limelight
 */
public class AngleAdjustAutoCommand extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private boolean isAngleAdjusted = false;

    public AngleAdjustAutoCommand(LowerSlide lowSlide, Limelight camera) {
        this.lowSlide = lowSlide;
        this.camera = camera;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAngleAdjusted = false;
        if (!camera.updateDetectorResult()) {
            isAngleAdjusted = true; // Skip if no detection
            return;
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        camera.updateDetectorResult();
        camera.updatePosition();
        // Processing angle for spinclaw
        double angle = camera.getAngle(); // -90 ~ 90
        angle = angle + ConfigVariables.Camera.ANGLE_OFFSET;
        lowSlide.spinclawSetPositionDeg(angle);
        packet.put("visionAdjust/angle", angle);
    }

    // This command must be interrupted after 500ms to stop
    @Override
    public long getTimeout() {
        return 2000; // Timeout after 1000ms
    }

    @Override
    public boolean isFinished() {
        return isAngleAdjusted;
    }

    @Override
    public void end(boolean interrupted) {
        isAngleAdjusted = true;
        camera.reset();
    }
}
