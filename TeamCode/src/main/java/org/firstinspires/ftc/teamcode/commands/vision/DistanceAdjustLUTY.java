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

import java.util.function.Supplier;

public class DistanceAdjustLUTY extends CommandBase {
    private final LowerSlide lowSlide;
    private final InterpLUT luty = new InterpLUT();
    // private final Gamepad gamepad1;
    private boolean isAdjusted = false;
    private double dy;
    Supplier<Double> tySupplier;
    private static final double SLIDE_MIN_CM = 0;
    private static final double SLIDE_MAX_CM = 40;

    // Variables for feedforward compensation
    private static final double CAMERA_DELAY = 0.1;
    private double prevDy = 0;
    private double dyVelocity = 0; // Change in dy per second
    private ElapsedTime velocityTimer = new ElapsedTime();
    private static final double VELOCITY_SMOOTHING = 0.7; // Smoothing factor for velocity calculation

    public DistanceAdjustLUTY(LowerSlide lowSlide, Supplier<Double> tySupplier) {
        if (tySupplier == null) {
            throw new IllegalArgumentException("tySupplier must not be null");
        }
        this.lowSlide = lowSlide;
        this.tySupplier = tySupplier;
        for (int i = 0; i < ConfigVariables.Camera.Y_DISTANCE_MAP_Y.length; i++) {
            luty.add(ConfigVariables.Camera.Y_DISTANCE_MAP_X[i], ConfigVariables.Camera.Y_DISTANCE_MAP_Y[i]);
        }
        luty.createLUT();
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        this.dy = safeGet(tySupplier);
        isAdjusted = false;
        // lowSlide.setPIDEnabled(false);
        velocityTimer.reset();
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Fetch fresh vision data every cycle
        this.dy = safeGet(tySupplier);
        packet.put("DistanceAdjustLUTY/dy_input", dy);
        packet.put("DistanceAdjustLUTY/isAdjusted", isAdjusted);

        if (dy == 0) {
            packet.put("DistanceAdjustLUTY/status", "NO_DY_VALUE");
            return;
        }

        packet.put("DistanceAdjustLUTY/status", "ADJUSTING");
        adjusty(dy, packet);
        isAdjusted = true;

        // if(gamepad1.dpad_up){
        // isAdjusted = true;
        // }

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
    }

    public void adjusty(double dy, TelemetryPacket packet) {
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

        // LUT range checking
        double minLut = luty.getMinX();
        double maxLut = luty.getMaxX();
        if (dy < minLut || dy > maxLut) {
            packet.put("vision/error", "dy out of LUT range: " + dy);
            return;
        }
        // Use the predicted position instead of current position (optional: swap dy for
        // predictedDy)
        double dycm = luty.get(dy); // or luty.get(predictedDy) for prediction
        double pos = lowSlide.getCurrentPositionCM() + dycm - luty.get(0) + ConfigVariables.Camera.Y_OFFSET;
        packet.put("vision/position set", pos);
        packet.put("vision/dycm", dycm);
        packet.put("vision/y0", luty.get(0));

        // Apply limits and set position
        if (pos > SLIDE_MAX_CM) {
            lowSlide.setPositionCM(SLIDE_MAX_CM);
        } else if (pos < SLIDE_MIN_CM) {
            lowSlide.setPositionCM(SLIDE_MIN_CM);
        } else {
            lowSlide.setPositionCM(pos);
        }
    }

    // Helper to safely get supplier values (null-safe, returns 0 if null)
    private double safeGet(Supplier<Double> supplier) {
        try {
            Double val = supplier.get();
            return val != null ? val : 0.0;
        } catch (Exception e) {
            return 0.0;
        }
    }
}
