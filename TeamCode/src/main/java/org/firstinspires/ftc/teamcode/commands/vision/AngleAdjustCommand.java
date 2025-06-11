package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.VisionStates;

/**
 * Command to perform vision-assisted adjustments using Limelight
 */
public class AngleAdjustCommand extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private final Gamepad gamepad1;
    private boolean isAngleAdjusted = false;
    private int currentState = VisionStates.Limelight.NO_TARGET;

    public AngleAdjustCommand(LowerSlide lowSlide, Limelight camera, Gamepad gamepad1) {
        this.lowSlide = lowSlide;
        this.camera = camera;
        this.gamepad1 = gamepad1;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAngleAdjusted = false;
        currentState = VisionStates.Limelight.NO_TARGET;
        if (!camera.updateDetectorResult()) {
            currentState = VisionStates.Limelight.ERROR;
            isAngleAdjusted = true; // Skip if no detection
            return;
        }
        currentState = VisionStates.Limelight.TARGET_DETECTED;
    }

    @Override
    public void execute(TelemetryPacket packet) {
        camera.updateDetectorResult();
        camera.updatePosition();

        // Processing angle for spinclaw
        double angle = camera.getAngle(); // -90 ~ 90
        currentState = VisionStates.Limelight.ADJUSTING_ANGLE;

        if (angle < -VisionStates.Limelight.ANGLE_THRESHOLD) {
            lowSlide.spinclawSetPositionDeg(VisionStates.Limelight.ANGLE_OFFSET);
        } else {
            lowSlide.spinclawSetPositionDeg(VisionStates.Limelight.ANGLE_OFFSET + 90);
        }

        packet.put("visionAdjust/angle", angle);
        packet.put("visionAdjust/state", getStateString());

        if (gamepad1.right_trigger > 0.5) {
            currentState = VisionStates.Limelight.ALIGNED;
            isAngleAdjusted = true;
        }
    }

    @Override
    public long getTimeout() {
        return 0; // No timeout
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

    private String getStateString() {
        switch (currentState) {
            case VisionStates.Limelight.NO_TARGET:
                return "No Target";
            case VisionStates.Limelight.TARGET_DETECTED:
                return "Target Detected";
            case VisionStates.Limelight.ADJUSTING_ANGLE:
                return "Adjusting Angle";
            case VisionStates.Limelight.ADJUSTING_DISTANCE:
                return "Adjusting Distance";
            case VisionStates.Limelight.ADJUSTING_POSITION:
                return "Adjusting Position";
            case VisionStates.Limelight.ALIGNED:
                return "Aligned";
            case VisionStates.Limelight.ERROR:
                return "Error";
            default:
                return "Unknown";
        }
    }
}
