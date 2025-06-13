package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

public class DistanceAdjustCommand extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private final PIDFController pidY;
//    private final Gamepad gamepad1;
    private boolean isAdjusted = false;

    public DistanceAdjustCommand(LowerSlide lowSlide, Limelight camera) { //Gamepad gamepad1
        this.lowSlide = lowSlide;
        this.camera = camera;
//        this.gamepad1 = gamepad1;
        this.pidY = new PIDFController(
                ConfigVariables.Camera.PID_KP,
                ConfigVariables.Camera.PID_KI,
                ConfigVariables.Camera.PID_KD,
                ConfigVariables.Camera.PID_KF);
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAdjusted = false;
        pidY.reset();
         lowSlide.setPIDEnabled(false);
        if (!camera.updateDetectorResult()) {
            isAdjusted = true; // Skip if no detection
            return;
        }
        camera.setColor(camera.getClassname());
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Adjust slide position using PID
        camera.updateDetectorResult();
        double dy = camera.getTy();
        double yPower = pidY.calculate(-dy);
        lowSlide.setSlidePower(yPower);

        // Add telemetry
        packet.put("visionAdjust/yDifference", dy);
        packet.put("visionAdjust/yPower", yPower);

        // Check if we're close enough
        // if (gamepad1.right_trigger > 0.5) {
        if (dy < ConfigVariables.Camera.DISTANCE_THRESHOLD) { //
            lowSlide.posNow(); // Hold current position
            lowSlide.setPIDEnabled(true);
            isAdjusted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isAdjusted;
    }

    @Override
    public void end(boolean interrupted) {
        lowSlide.setPIDEnabled(true);
        if (interrupted) {
            lowSlide.stop();
        }
        camera.reset();
    }
}
