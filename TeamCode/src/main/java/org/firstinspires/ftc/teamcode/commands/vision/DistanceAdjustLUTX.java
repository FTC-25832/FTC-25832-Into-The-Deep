package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;

import java.util.function.Supplier;

public class DistanceAdjustLUTX extends CommandBase {
    private final InterpLUT lutx = new InterpLUT();
    // private final InterpLUT lutratio = new InterpLUT();
    private final MecanumDrive drive;
    private double tx, py, ty, px;
    private boolean isAdjusted = false;
    private Action moveAction = null;
    Supplier<Double> txSupplier, pxSupplier, pySupplier, tySupplier;
    private final Runnable disableDriveControl;
    private final Runnable enableDriveControl;
    private static final int CAMERA_CX_INDEX = 2; // Index for camera matrix center x
    private static final int CAMERA_FX_INDEX = 0; // Index for camera matrix focal length x
    private static final int CAMERA_MATRIX_ROW = 0; // Row for camera matrix
    private static final double TIMEOUT_SECONDS = 3.0; // Timeout for movement
    private long actionStartTime = 0; // For timeout

    public DistanceAdjustLUTX(MecanumDrive drive, Supplier<Double> txSupplier, Supplier<Double> tySupplier,
            Supplier<Double> pxSupplier, Supplier<Double> pySupplier, Runnable disableDriveControl,
            Runnable enableDriveControl) {
        if (txSupplier == null || tySupplier == null || pxSupplier == null || pySupplier == null) {
            throw new IllegalArgumentException("Suppliers must not be null");
        }
        for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
            lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i], ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
        }
        // for (int i = 0; i < ConfigVariables.Camera.XYRATIO_MAP_Y.length; i++) {
        // lutratio.add(ConfigVariables.Camera.XYRATIO_MAP_X[i],
        // ConfigVariables.Camera.XYRATIO_MAP_Y[i]);
        // }
        lutx.createLUT();
        // lutratio.createLUT();
        this.pxSupplier = pxSupplier;
        this.pySupplier = pySupplier;
        this.tySupplier = tySupplier;
        this.txSupplier = txSupplier;
        this.drive = drive;
        this.disableDriveControl = disableDriveControl;
        this.enableDriveControl = enableDriveControl;
    }

    @Override
    public void initialize() {
        // Fetch latest vision data
        this.tx = safeGet(txSupplier);
        this.px = safeGet(pxSupplier);
        this.py = safeGet(pySupplier);
        this.ty = safeGet(tySupplier);
        isAdjusted = false;
        moveAction = null;
        actionStartTime = 0;
        disableDriveControl.run();
    }

    public double pixToAngle(double px) { // return rad
        return Math.atan((px - ConfigVariables.Camera.CAMERA_MATRIX[CAMERA_MATRIX_ROW][CAMERA_CX_INDEX])
                / ConfigVariables.Camera.CAMERA_MATRIX[CAMERA_MATRIX_ROW][CAMERA_FX_INDEX]);
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Fetch fresh vision data every cycle
        this.tx = safeGet(txSupplier);
        this.px = safeGet(pxSupplier);
        this.py = safeGet(pySupplier);
        this.ty = safeGet(tySupplier);
        packet.put("DistanceAdjustLUTX/dx_input", tx);
        packet.put("DistanceAdjustLUTX/py_input", py);
        packet.put("DistanceAdjustLUTX/isAdjusted", isAdjusted);
        packet.put("DistanceAdjustLUTX/hasActiveAction", moveAction != null);

        // If we have an active movement action, run it
        if (moveAction != null) {
            // Timeout check
            if (actionStartTime == 0)
                actionStartTime = System.currentTimeMillis();
            double elapsed = (System.currentTimeMillis() - actionStartTime) / 1000.0;
            boolean timedOut = elapsed > TIMEOUT_SECONDS;
            boolean actionDone = !moveAction.run(packet);

            if (actionDone || timedOut) {
                isAdjusted = true;
                moveAction = null;
                packet.put("DistanceAdjustLUTX/actionStatus", timedOut ? "timed out" : "completed");
            } else {
                drive.updatePoseEstimate();
                return; // Continue running current action
            }
        } else {
            if (tx == 0 || ty == 0) {
                isAdjusted = true;
                packet.put("DistanceAdjustLUTX/status", "NO_DX_VALUE");
                return;
            }

            packet.put("DistanceAdjustLUTX/status", "ADJUSTING");
            final double[] vanishingPoint = ConfigVariables.Camera.VANISHING_POINT;
            // Division by zero check for vertical line
            double denominator = (vanishingPoint[1] - py);
            if (Math.abs(denominator) < 1e-6) {
                packet.put("vision/error", "Vertical line: vanishingPoint[1] == py");
                isAdjusted = true;
                return;
            }
            final double m = (vanishingPoint[0] - px) / denominator;
            final double b = px - m * py;
            packet.put("vision/m", m);
            packet.put("vision/b", b);
            final double crosshairY = ConfigVariables.Camera.CROSSHAIR_Y_PX;
            final double newpx = m * crosshairY + b;
            packet.put("vision/newpx", newpx);
            // tx = (px-cx)/fx
            final double fx1 = pixToAngle(newpx);
            final double fx2 = pixToAngle(ConfigVariables.Camera.CROSSHAIR_X_PX);
            final double newtx = Math.toDegrees(fx1 - fx2);
            packet.put("vision/newtx", newtx);
            tx = newtx;
            adjustx(tx, packet);
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
            // Explicitly nullify moveAction for cleanup
            moveAction = null;
        }
    }

    public void adjustx(double dx, TelemetryPacket packet) {
        // LUT range checking
        double minLut = lutx.getMinX();
        double maxLut = lutx.getMaxX();
        if (dx < minLut || dx > maxLut) {
            packet.put("vision/error", "dx out of LUT range: " + dx);
            isAdjusted = true;
            moveAction = null;
            return;
        }
        double dxcm = lutx.get(dx);
        packet.put("vision/dxcm", dxcm);

        double adjustmentNeededinch = dxcm / 2.54;
        Pose2d startpose = drive.localizer.getPose();
        double heading = startpose.heading.toDouble();
        // robot centric to field centric
        Vector2d endpose = new Vector2d(
                startpose.position.x + adjustmentNeededinch * Math.sin(heading),
                startpose.position.y - adjustmentNeededinch * Math.cos(heading));
        // Create the action
        moveAction = drive.actionBuilder(startpose).strafeToLinearHeading(endpose, Rotation2d.fromDouble(heading))
                .build();
        // Run the action first time
        drive.updatePoseEstimate();
        moveAction.run(packet);
        actionStartTime = System.currentTimeMillis();
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
