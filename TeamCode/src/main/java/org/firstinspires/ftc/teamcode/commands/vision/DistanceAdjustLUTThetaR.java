package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;

import java.util.function.Supplier;

public class DistanceAdjustLUTThetaR extends CommandBase {
    private final InterpLUT lutx = new InterpLUT();
    private final InterpLUT luty = new InterpLUT();
    private final LowerSlide lowSlide;
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

    public DistanceAdjustLUTThetaR(LowerSlide lowslide, MecanumDrive drive, Supplier<Double> txSupplier,
            Supplier<Double> tySupplier, Supplier<Double> pxSupplier, Supplier<Double> pySupplier,
            Runnable disableDriveControl,
            Runnable enableDriveControl) {
        if (txSupplier == null || tySupplier == null || pxSupplier == null || pySupplier == null) {
            throw new IllegalArgumentException("Suppliers must not be null");
        }
        for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
            lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i], ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
        }
        lutx.createLUT();
        for (int i = 0; i < ConfigVariables.Camera.Y_DISTANCE_MAP_Y.length; i++) {
            luty.add(ConfigVariables.Camera.Y_DISTANCE_MAP_X[i], ConfigVariables.Camera.Y_DISTANCE_MAP_Y[i]);
        }
        luty.createLUT();
        this.lowSlide = lowslide;
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
            lowSlide.updatePID();
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
            final double fx1 = pixToAngle(newpx);
            final double fx2 = pixToAngle(ConfigVariables.Camera.CROSSHAIR_X_PX);
            final double newtx = Math.toDegrees(fx1 - fx2);
            packet.put("vision/newtx", newtx);
            tx = newtx;
            adjust(tx, ty, packet);
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
            drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                    new Vector2d(0, 0), 0));
            moveAction = null;
        }
    }

    public void adjust(double tx, double ty, TelemetryPacket packet) {
        // LUT range checking
        double minLutX = lutx.getMinX();
        double maxLutX = lutx.getMaxX();
        double minLutY = luty.getMinX();
        double maxLutY = luty.getMaxX();
        if (tx < minLutX || tx > maxLutX) {
            packet.put("vision/error", "tx out of LUT range: " + tx);
            isAdjusted = true;
            moveAction = null;
            return;
        }
        if (ty < minLutY || ty > maxLutY) {
            packet.put("vision/error", "ty out of LUT range: " + ty);
            isAdjusted = true;
            moveAction = null;
            return;
        }
        double dxcm = lutx.get(tx);
        packet.put("vision/dxcm", dxcm);
        double dycm = luty.get(ty);
        packet.put("vision/dycm", dycm);
        double lowslideExtendcm = lowSlide.getCurrentPositionCM();
        double dyrobot = dycm + ConfigVariables.Camera.HALF_ROBOT_LENGTH + lowslideExtendcm;
        double theta = Math.atan2(-dxcm, dyrobot);
        double r = Math.sqrt(dxcm * dxcm + dyrobot * dyrobot) - ConfigVariables.Camera.HALF_ROBOT_LENGTH
                - lowslideExtendcm;
        packet.put("vision/theta", Math.toDegrees(theta));
        packet.put("vision/r", r);

        double pos = lowSlide.getCurrentPositionCM() + r - luty.get(0) + ConfigVariables.Camera.Y_OFFSET;
        if (pos > 45) {
            lowSlide.setPositionCM(45);
        } else if (pos < 0) {
            lowSlide.setPositionCM(0);
        } else {
            lowSlide.setPositionCM(pos);
        }
        packet.put("vision/position set", pos);
        packet.put("vision/y0", luty.get(0));
        Pose2d startpose = drive.localizer.getPose();
        moveAction = drive.actionBuilder(startpose).turnTo(startpose.heading.toDouble() + theta).build();
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
