package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
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
    private final InterpLUT lutx = new InterpLUT();
    private final MecanumDrive drive;
    private final Gamepad gamepad1;
    private final Limelight camera;
    private double dx, dy;
    private boolean isAdjusted = false;
    private Action moveAction = null;
    private final Runnable disableDriveControl;
    private final Runnable enableDriveControl;
    private final boolean isTeleop;

    public DistanceAdjustLUTX(MecanumDrive drive, Limelight camera, Runnable disableDriveControl,
            Runnable enableDriveControl) {
        this(null, drive, camera, disableDriveControl, enableDriveControl);
    }

    public DistanceAdjustLUTX(Gamepad gamepad1, MecanumDrive drive, Limelight camera, Runnable disableDriveControl,
            Runnable enableDriveControl) {
        for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
            lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i], ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
        }
        lutx.createLUT();
        this.camera = camera;
        this.gamepad1 = gamepad1;
        this.drive = drive;
        this.disableDriveControl = disableDriveControl;
        this.enableDriveControl = enableDriveControl;
        this.isTeleop = gamepad1 != null;
    }

    @Override
    public void initialize() {
        isAdjusted = false;
        moveAction = null;
        disableDriveControl.run();
    }

    @Override
    public void execute(TelemetryPacket packet) {
        camera.updateDetectorResult();
        dx = camera.getTx();
        dy = camera.getTy();

        packet.put("vision/x", "running");

        // If we have an active movement action, run it
        if (moveAction != null) {
            // Check if action is done or timed out
            boolean actionDone = !moveAction.run(packet);

            if (actionDone) {
                isAdjusted = true;
                moveAction = null;
                packet.put("vision/x", actionDone ? "completed" : "timed out");
            } else {
                drive.updatePoseEstimate();
                return; // Continue running current action
            }
        } else {
            if (dx == 0) {
                isAdjusted = true;
                if (isTeleop) {
                    gamepad1.rumble(100);
                }
                packet.put("vision/x", "no target");
                return;
            }
            // tx: +11.11°
            // ty: +31.91°
            final double gradient = 7.3 / 23.7;
            dx = dx - dy * gradient;
            adjustx(dx, packet);
        }

        if (isTeleop && gamepad1.dpad_up) {
            isAdjusted = true;
            moveAction = null;
            packet.put("vision/x", "completed by manual");
        }
    }

    @Override
    public boolean isFinished() {
        return isAdjusted && moveAction == null;
    }

    @Override
    public void end(boolean interrupted) {
        enableDriveControl.run();
        // Stop any ongoing movement if interrupted
        if (interrupted && moveAction != null) {
            // Set drive powers to zero to stop movement
            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                    new Vector2d(0, 0), 0));
        }
    }

    public void adjustx(double dx, TelemetryPacket packet) {
        packet.put("vision/rawTx", dx);

        double dxcm = lutx.get(dx);
        packet.put("vision/dxcm", dxcm);

        // Calculate how much we need to adjust
        double adjustmentNeeded = dxcm - lutx.get(0);
        packet.put("vision/adjustmentNeeded", adjustmentNeeded);

        if (Math.abs(adjustmentNeeded) > 0.25) { // Only move if adjustment is significant
            double adjustmentNeededinch = adjustmentNeeded / 2.54;
            Pose2d startpose = drive.localizer.getPose();
            double heading = startpose.heading.toDouble();
            // robot centric to field centric
            Vector2d endpose = new Vector2d(
                    startpose.position.x + adjustmentNeededinch * Math.sin(heading),
                    startpose.position.y - adjustmentNeededinch * Math.cos(heading));

            // Create the action
            moveAction = drive.actionBuilder(startpose)
                    .strafeToConstantHeading(endpose)
                    .build();

            // Run the action first time
            drive.updatePoseEstimate();
            moveAction.run(packet);
        } else {
            isAdjusted = true; // No significant adjustment needed
        }
    }
}