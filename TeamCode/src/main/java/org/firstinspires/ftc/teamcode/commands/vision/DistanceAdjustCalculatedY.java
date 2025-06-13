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

public class DistanceAdjustCalculatedY extends CommandBase {
    private final LowerSlide lowSlide;
    // private final Gamepad gamepad1;
    private boolean isAdjusted = false;
    private double dy;
    Supplier<Double> dySupplier;

    public DistanceAdjustCalculatedY(LowerSlide lowSlide, Supplier<Double> dySupplier) {
        this.lowSlide = lowSlide;
        this.dySupplier = dySupplier;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        this.dy = dySupplier.get();
        isAdjusted = false;
    }

    @Override
    public void execute(TelemetryPacket packet) {
        packet.put("DistanceAdjustCalculatedY/dy_input", dy);
        packet.put("DistanceAdjustCalculatedY/isAdjusted", isAdjusted);

        // if(Math.abs(lowSlide.pidfController.lastError) > 20) return;
        if (dy == 0) {
            packet.put("DistanceAdjustCalculatedY/status", "NO_DY_VALUE");
            return;
        }

        packet.put("DistanceAdjustCalculatedY/status", "ADJUSTING");
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
        double pos = lowSlide.getCurrentPositionCM() + dy - ConfigVariables.Camera.CAMERA_DISTANCE + ConfigVariables.Camera.Y_OFFSET;
        packet.put("vision/position set", pos);
        packet.put("vision/dycm", dy);

        // Apply limits and set position
        if (pos > 45) {
            lowSlide.setPositionCM(45);
        } else if (pos < 0) {
            lowSlide.setPositionCM(0);
        } else {
            lowSlide.setPositionCM(pos);
        }
    }
}
