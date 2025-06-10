package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;
import org.firstinspires.ftc.teamcode.utils.timing.Timeout;

public class DistanceAdjustLUTX extends CommandBase {
    private final Limelight camera;
    private final InterpLUT lutx = new InterpLUT();
    private final MecanumDrive drive;
    private final Gamepad gamepad1;
    private final CommandScheduler scheduler;
    private boolean isAdjusted = false;

    private static final double VELOCITY_SMOOTHING = 0.7; // Smoothing factor for velocity calculation

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public DistanceAdjustLUTX(Limelight camera, Gamepad gamepad1, MecanumDrive drive, CommandScheduler scheduler) {
        this.camera = camera;
        for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
            lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i], ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
        }
        lutx.createLUT();
        this.gamepad1 = gamepad1;
        this.scheduler = scheduler;
        this.drive = drive;
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
        packet.put("vision/x", "running");
        //        if(Math.abs(lowSlide.pidfController.lastError) > 20) return;
        if(camera.updateDetectorResult()){
            double dx = camera.getTx();
            if(dx == 0){
                return;
            }
            adjustx(dx, packet);
            isAdjusted = true;

        }
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

        camera.reset();
    }

    public void adjustx(double dx, TelemetryPacket packet){
        packet.put("vision/rawTx", dx);

        double dxcm = lutx.get(dx);
        packet.put("vision/dxcm", dxcm);

        // Calculate how much we need to adjust
        double adjustmentNeeded = dxcm - lutx.get(0);
        packet.put("vision/adjustmentNeeded", adjustmentNeeded);

        if (Math.abs(adjustmentNeeded) > 0.5) { // Only move if adjustment is significant
            Pose2d startpose = drive.localizer.getPose();
            double heading = startpose.heading.toDouble();
            // robot centric to field centric
            Vector2d endpose = new Vector2d(
                    startpose.position.x + adjustmentNeeded * Math.cos(heading),
                    startpose.position.y + adjustmentNeeded * Math.sin(heading)
            );
            scheduler.schedule(new ActionCommand(drive.actionBuilder(startpose).strafeToConstantHeading(endpose).build()));
        }
    }
}