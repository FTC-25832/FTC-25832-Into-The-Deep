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

import java.util.function.Supplier;

public class DistanceAdjustLUTX extends CommandBase {
    private final InterpLUT lutx = new InterpLUT();
    // private final InterpLUT lutratio = new InterpLUT();
    private final MecanumDrive drive;
    private double dx, dy;
    private boolean isAdjusted = false;
    private Action moveAction = null;
    Supplier<Double> txSupplier, tySupplier;
    private final Runnable disableDriveControl;
    private final Runnable enableDriveControl;

    public DistanceAdjustLUTX(MecanumDrive drive, Supplier<Double> txSupplier, Supplier<Double> tySupplier, Runnable disableDriveControl,
            Runnable enableDriveControl) {
        for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
            lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i], ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
        }
        // for (int i = 0; i < ConfigVariables.Camera.XYRATIO_MAP_Y.length; i++) {
        // lutratio.add(ConfigVariables.Camera.XYRATIO_MAP_X[i],
        // ConfigVariables.Camera.XYRATIO_MAP_Y[i]);
        // }
        lutx.createLUT();
        // lutratio.createLUT();
        this.tySupplier = tySupplier;
        this.txSupplier = txSupplier;
        this.drive = drive;
        this.disableDriveControl = disableDriveControl;
        this.enableDriveControl = enableDriveControl;
    }

    @Override
    public void initialize() {
        this.dx = txSupplier.get();
        this.dy = tySupplier.get();
        isAdjusted = false;
        moveAction = null;
        disableDriveControl.run();
    }

    @Override
    public void execute(TelemetryPacket packet) {
        packet.put("DistanceAdjustLUTX/dx_input", dx);
        packet.put("DistanceAdjustLUTX/dy_input", dy);
        packet.put("DistanceAdjustLUTX/isAdjusted", isAdjusted);
        packet.put("DistanceAdjustLUTX/hasActiveAction", moveAction != null);

        // If we have an active movement action, run it
        if (moveAction != null) {
            // Check if action is done or timed out
            boolean actionDone = !moveAction.run(packet);

            if (actionDone) {
                isAdjusted = true;
                moveAction = null;
                packet.put("DistanceAdjustLUTX/actionStatus", actionDone ? "completed" : "timed out");
            } else {
                drive.updatePoseEstimate();
                return; // Continue running current action
            }
        } else {
            if (dx == 0) {
                isAdjusted = true;
                packet.put("DistanceAdjustLUTX/status", "NO_DX_VALUE");
                return;
            }

            packet.put("DistanceAdjustLUTX/status", "ADJUSTING");
            // final double gradient = lutratio.get(dy);
            // dx = dx - dy * gradient;
            // Δtx = 180/pi arctan(tan(ty*pi/180)*gradientpx))

            final double gradientpx = ConfigVariables.Camera.XYPIXELRATIO;
            final double ddx = Math.toDegrees(Math.atan(Math.tan(Math.toRadians(dy)) * gradientpx));
            packet.put("vision/ddx", ddx);
            dx = dx - ddx;
            adjustx(dx, packet);
        }

        // if (gamepad1.dpad_up) {
        // isAdjusted = true;
        // moveAction = null;
        // packet.put("vision/x", "completed by manual");
        // }
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
        packet.put("vision/dx", dx);

        double dxcm = lutx.get(dx);
        packet.put("vision/dxcm", dxcm);

        // Calculate how much we need to adjust
        double adjustmentNeeded = dxcm - lutx.get(0);
        packet.put("vision/adjustmentNeeded", adjustmentNeeded);

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
    }
}
