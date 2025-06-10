package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;
import org.firstinspires.ftc.teamcode.utils.timing.Timeout;

public class DistanceAdjustLUTX extends CommandBase {
    private final Limelight camera;
    private final InterpLUT lutx = new InterpLUT();
    private final MecanumDrive drive;
    private final Gamepad gamepad1;
    private boolean isAdjusted = false;
    private Action moveAction = null;
    private ElapsedTime actionTimer = new ElapsedTime();
    private static final double ACTION_TIMEOUT = 1.5; // 1.5 seconds timeout for movement

    private static final double VELOCITY_SMOOTHING = 0.7; // Smoothing factor for velocity calculation

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public DistanceAdjustLUTX(Limelight camera, Gamepad gamepad1, MecanumDrive drive) {
        this.camera = camera;
        for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
            lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i], ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
        }
        lutx.createLUT();
        this.gamepad1 = gamepad1;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        isAdjusted = false;
        moveAction = null;
        if (!camera.updateDetectorResult()) {
            isAdjusted = true; // Skip if no detection
            return;
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        packet.put("vision/x", "running");

        // If we have an active movement action, run it
        if (moveAction != null) {
            // Check if action is done or timed out
            boolean actionDone = !moveAction.run(packet);
            boolean timedOut = actionTimer.seconds() > ACTION_TIMEOUT;

            if (actionDone || timedOut) {
                moveAction = null;
                isAdjusted = true;
                packet.put("vision/adjustment", actionDone ? "completed" : "timed out");
            } else {
                drive.updatePoseEstimate();
                return; // Continue running current action
            }
        }

        // Only detect new adjustments if no active movement
        if (moveAction == null && camera.updateDetectorResult()) {
            double dx = camera.getTx();
            if (dx != 0) {
                adjustx(dx, packet);
            }
        }

        if (gamepad1.dpad_up) {
            isAdjusted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isAdjusted && moveAction == null;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop any ongoing movement if interrupted
        if (interrupted && moveAction != null) {
            // Set drive powers to zero to stop movement
            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                    new Vector2d(0, 0), 0));
        }
        camera.reset();
    }

    public void adjustx(double dx, TelemetryPacket packet) {
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
                    startpose.position.x - adjustmentNeeded * Math.sin(heading),
                    startpose.position.y - adjustmentNeeded * Math.cos(heading)
            );

            // Create the action
            moveAction = drive.actionBuilder(startpose)
                    .strafeToConstantHeading(endpose)
                    .build();

            // Reset the timer for timeout tracking
            actionTimer.reset();

            // Run the action first time
            drive.updatePoseEstimate();
            moveAction.run(packet);
        } else {
            isAdjusted = true; // No significant adjustment needed
        }
    }
}