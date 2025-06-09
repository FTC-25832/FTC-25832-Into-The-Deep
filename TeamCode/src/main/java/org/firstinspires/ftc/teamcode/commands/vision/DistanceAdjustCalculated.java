package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

public class DistanceAdjustCalculated extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private final Gamepad gamepad1;
    private boolean isAdjusted = false;
    public DistanceAdjustCalculated(LowerSlide lowSlide, Limelight camera, Gamepad gamepad1) {
        this.lowSlide = lowSlide;
        this.camera = camera;
        this.gamepad1 = gamepad1;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAdjusted = false;
//        lowSlide.setPIDEnabled(false);
        if (!camera.updateDetectorResult()) {
            isAdjusted = true; // Skip if no detection
            return;
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        camera.updateDetectorResult();
        camera.updatePosition();
        // processing position
        double dy = camera.getWorldy();
        lowSlide.setPositionCM(lowSlide.getCurrentPositionCM() + dy - ConfigVariables.Camera.CAMERA_DISTANCE);
//        if(dy - ConfigVariables.Camera.CAMERA_DISTANCE < ConfigVariables.Camera.DISTANCE_THRESHOLD){
//            isAdjusted = true;
//        }
        if(gamepad1.dpad_up){
            isAdjusted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isAdjusted;
    }

    @Override
    public void end(boolean interrupted) {
        lowSlide.stop();
        camera.reset();
    }
}
