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
public class AngleAdjustCommand extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private boolean isAngleAdjusted = false;
    private double angleAccum = 0;
    private double angleNum = 1;

    public AngleAdjustCommand(LowerSlide lowSlide, Limelight camera) {
        this.lowSlide = lowSlide;
        this.camera = camera;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAngleAdjusted = false;
        angleAccum = 0;
        angleNum = 1;

        if (!camera.updateDetectorResult()) {
            isAngleAdjusted = true; // Skip if no detection
            return;
        }

        camera.switchtoPython();
        camera.setColor(camera.getClassname());
    }

    @Override
    public void execute(TelemetryPacket packet) {
            // Processing angle for spinclaw
            double angle = camera.getAngle(); // -90 ~ 90
            angle = angle + ConfigVariables.Camera.ANGLE_OFFSET;
            angleAccum += angle;
            angleNum += 1;
            packet.put("visionAdjust/angle", angle);
    }

    // This command must be interrupted after 500ms to stop
    @Override
    public long getTimeout() {
        return 500; // Timeout after 500ms
    }

    @Override
    public boolean isFinished() {
        return isAngleAdjusted;
    }

    @Override
    public void end(boolean interrupted) {
        double averageAngle = angleAccum / angleNum;
        lowSlide.spinclawSetPositionDeg(averageAngle);
        isAngleAdjusted = true;
        camera.reset();
    }
}
