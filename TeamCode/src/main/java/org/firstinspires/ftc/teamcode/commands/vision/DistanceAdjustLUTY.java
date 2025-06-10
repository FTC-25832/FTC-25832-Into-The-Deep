package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;
import org.firstinspires.ftc.teamcode.utils.timing.Timeout;

public class DistanceAdjustLUTY extends CommandBase {
    private final LowerSlide lowSlide;
    private final Limelight camera;
    private final InterpLUT luty = new InterpLUT();
    private final Gamepad gamepad1;
    private boolean isAdjusted = false;

    // Variables for feedforward compensation
    private static final double CAMERA_DELAY = 0.1;
    private double prevDy = 0;
    private double dyVelocity = 0; // Change in dy per second
    private ElapsedTime velocityTimer = new ElapsedTime();
    private static final double VELOCITY_SMOOTHING = 0.7; // Smoothing factor for velocity calculation

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public DistanceAdjustLUTY(LowerSlide lowSlide, Limelight camera, Gamepad gamepad1) {
        this.lowSlide = lowSlide;
        this.camera = camera;
        for (int i = 0; i < ConfigVariables.Camera.Y_DISTANCE_MAP_Y.length; i++) {
            luty.add(ConfigVariables.Camera.Y_DISTANCE_MAP_X[i], ConfigVariables.Camera.Y_DISTANCE_MAP_Y[i]);
        }
        luty.createLUT();
        this.gamepad1 = gamepad1;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAdjusted = false;
//        lowSlide.setPIDEnabled(false);
        if (!camera.resultAvailable) {
            isAdjusted = true; // Skip if no detection
            return;
        }
        velocityTimer.reset();
    }

    @Override
    public void execute(TelemetryPacket packet) {
//        if(Math.abs(lowSlide.pidfController.lastError) > 20) return;
        double dy = camera.getTy();
        double dx = camera.getTx();
        if(dy == 0 || dx == 0){
            return;
        }
        adjusty(dy, packet);
        isAdjusted = true;

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
        if (interrupted) {
            lowSlide.stop();
            return;
        }

        camera.reset();
    }

    public void adjusty(double dy, TelemetryPacket packet){
        // Calculate velocity (change in dy per second)
        double dt = velocityTimer.seconds();
        velocityTimer.reset();
        if (dt > 0) {
            double instantVelocity = (dy - prevDy) / dt;
            dyVelocity = VELOCITY_SMOOTHING * instantVelocity + (1 - VELOCITY_SMOOTHING) * dyVelocity;
        }
        prevDy = dy;

        packet.put("vision/rawTy", dy);

        // Apply feedforward to predict position after delay
        double predictedDy = dy + (dyVelocity * CAMERA_DELAY);
        packet.put("vision/predictedTy", predictedDy);
        packet.put("vision/velocity", dyVelocity);

        // Use the predicted position instead of current position
        double dycm = luty.get(dy);
        double pos = lowSlide.getCurrentPositionCM() + dycm - luty.get(0);
        packet.put("vision/position set", pos);
        packet.put("vision/dycm", dycm);
        packet.put("vision/y0", luty.get(0));


        // Apply limits and set position
        if(pos > 45){
            lowSlide.setPositionCM(45);
        } else if (pos < 0){
            lowSlide.setPositionCM(0);
        } else {
            lowSlide.setPositionCM(pos);
        }
    }
}