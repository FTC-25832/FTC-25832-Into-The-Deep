package org.firstinspires.ftc.teamcode.commands.vision;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

/**
 * Command to perform vision-assisted adjustments using Limelight
 */
public class AngleAdjustCommand extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private List<Double> angles;
    private boolean isAngleAdjusted = false;

    public AngleAdjustCommand(LowerSlide lowSlide, Limelight camera) { // Gamepad gamepad1
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
        angles.add(angle);
        packet.put("visionAdjust/angle", angle);
        if(angles.size()>=ConfigVariables.Camera.ANGLE_MAXNUM){
            isAngleAdjusted = true;
        }

    }

    // This command must be interrupted after 500ms to stop
    @Override
    public long getTimeout() {
        return 0; // Timeout after 1000ms
    }

    @Override
    public boolean isFinished() {
        return isAngleAdjusted;
    }

    @Override
    public void end(boolean interrupted) {
        isAngleAdjusted = true;
        if(angles.stream().mapToDouble((a)->a).sum()/angles.size() < -45){
            lowSlide.spinclawSetPositionDeg(ConfigVariables.Camera.CLAW_90+90);
        } else {
            lowSlide.spinclawSetPositionDeg(ConfigVariables.Camera.CLAW_90);
        }
        camera.reset();
    }
}