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
    private long startTime; // For timeout

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
        startTime = System.currentTimeMillis();

        if (!camera.updateDetectorResult()) {
            isAngleAdjusted = true; // Skip if no detection
            // gamepad1.rumble(200); // Removed gamepad dependency
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
        if (angleNum > ConfigVariables.Camera.ANGLE_MAXNUM) {
            lowSlide.spinclawSetPositionDeg(angleAccum / angleNum);
            angleAccum = 0; // Reset after adjustment
            angleNum = 1;   // Reset after adjustment
            // isAngleAdjusted = true; // Consider finishing after one successful adjustment cycle
        }
        // Removed: if (gamepad1.right_trigger > 0.5 || gamepad1.dpad_up) { isAngleAdjusted = true; }
    }

    // This command must be interrupted after 500ms to stop (Original comment)
    // Implementing timeout via isFinished()
    @Override
    public long getTimeout() {
        return 500; // Informational, actual logic in isFinished
    }

    @Override
    public boolean isFinished() {
        if (isAngleAdjusted) { // Handles case where camera detection fails
            return true;
        }
        // Finish if timeout is reached
        return (System.currentTimeMillis() - startTime) >= 500;
    }

    @Override
    public void end(boolean interrupted) {
        camera.switchtoNeural();
        camera.reset();
    }
}
