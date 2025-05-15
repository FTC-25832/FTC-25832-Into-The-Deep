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
    private final PIDFController pidY;
    private boolean isAdjusted = false;
    private double dyAccum = 0;
    private int dyNum = 0;
    public DistanceAdjustCalculated(LowerSlide lowSlide, Limelight camera) {
        this.lowSlide = lowSlide;
        this.camera = camera;
        this.pidY = new PIDFController(
                ConfigVariables.Camera.PID_KP,
                ConfigVariables.Camera.PID_KI,
                ConfigVariables.Camera.PID_KD,
                ConfigVariables.Camera.PID_KF);
        dyAccum = 0;
        dyNum = 0;
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isAdjusted = false;
        pidY.reset();
//        lowSlide.setPIDEnabled(false);
        camera.switchtoNeural();
        if (!camera.updateDetectorResult()) {
            isAdjusted = true; // Skip if no detection
            return;
        }
        camera.setColor(camera.getClassname());
    }

    @Override
    public void execute(TelemetryPacket packet) {
        camera.updateDetectorResult();
        // processing position
        double dy = camera.getY();
        if(dy==0){
            return;
        }
        dyAccum += dy;
        dyNum += 1;
        if(dyNum > ConfigVariables.Camera.YACCUM_MAXNUM){
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
        if(dyNum==0){
            return;
        }
        double current = lowSlide.getCurrentPositionCM();
        double averageY = dyAccum / dyNum;
        averageY = Math.min(averageY, ConfigVariables.Camera.DISTANCE_MAP.length - 1);
        double position;
        if(averageY<0){
            averageY = -averageY;
            position = ConfigVariables.Camera.DISTANCE_MAP_NEGATIVE[(int) Math.floor(averageY)] + (averageY - Math.floor(averageY)) *
                    (ConfigVariables.Camera.DISTANCE_MAP_NEGATIVE[(int) Math.floor(averageY) + 1] - ConfigVariables.Camera.DISTANCE_MAP_NEGATIVE[(int) Math.floor(averageY)]);
        } else{
            position = ConfigVariables.Camera.DISTANCE_MAP[(int) Math.floor(averageY)] + (averageY - Math.floor(averageY)) *
                    (ConfigVariables.Camera.DISTANCE_MAP[(int) Math.floor(averageY) + 1] - ConfigVariables.Camera.DISTANCE_MAP[(int) Math.floor(averageY)]);
        }
        lowSlide.setPositionCM(current + position - ConfigVariables.Camera.CLAW_DISTANCE);
        camera.reset();
    }
}
