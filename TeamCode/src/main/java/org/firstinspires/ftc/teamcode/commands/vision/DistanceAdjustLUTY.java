package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;

import java.util.function.Supplier;

public class DistanceAdjustLUTY extends CommandBase {
    // Variables for feedforward compensation
    private static final double CAMERA_DELAY = 0.1;
    private static final double VELOCITY_SMOOTHING = 0.7; // Smoothing factor for velocity calculation
    private final LowerSlide lowSlide;
    private final InterpLUT luty = new InterpLUT();
    Supplier<Double> tySupplier;
    // private final Gamepad gamepad1;
    private boolean isAdjusted = false;
    private double dy;
    private double prevDy = 0;
    private double dyVelocity = 0; // Change in dy per second
    private ElapsedTime velocityTimer = new ElapsedTime();

    public DistanceAdjustLUTY(LowerSlide lowSlide, Supplier<Double> tySupplier) {
        this.lowSlide = lowSlide;
        this.tySupplier = tySupplier;
        for (int i = 0; i < ConfigVariables.Camera.Y_DISTANCE_MAP_Y.length; i++) {
            luty.add(ConfigVariables.Camera.Y_DISTANCE_MAP_X[i], ConfigVariables.Camera.Y_DISTANCE_MAP_Y[i]);
        }
        luty.createLUT();
        addRequirement(lowSlide);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void initialize() {
        this.dy = tySupplier.get();
        isAdjusted = false;
        // lowSlide.setPIDEnabled(false);
        velocityTimer.reset();
    }

    @Override
    public void execute(TelemetryPacket packet) {
        packet.put("DistanceAdjustLUTY/dy_input", dy);
        packet.put("DistanceAdjustLUTY/isAdjusted", isAdjusted);

        // if(Math.abs(lowSlide.pidfController.lastError) > 20) return;
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

        // Use the predicted position instead of current position
        double dycm = luty.get(dy);
        double pos = lowSlide.getCurrentPositionCM() + dycm - luty.get(0) + ConfigVariables.Camera.Y_OFFSET;
        packet.put("vision/position set", pos);
        packet.put("vision/dycm", dycm);
        packet.put("vision/y0", luty.get(0));

        // Apply limits and set position
        if (pos > 40) {
            lowSlide.setPositionCM(40);
        } else if (pos < 0) {
            lowSlide.setPositionCM(0);
        } else {
            lowSlide.setPositionCM(pos);
        }
    }
}
