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

    public DistanceAdjustLUTThetaR(LowerSlide lowslide, MecanumDrive drive, Supplier<Double> txSupplier, Supplier<Double> tySuplier, Supplier<Double> pxSupplier, Supplier<Double> pySupplier, Runnable disableDriveControl,
                                   Runnable enableDriveControl) {
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
        this.tySupplier = tySuplier;
        this.txSupplier = txSupplier;
        this.drive = drive;
        this.disableDriveControl = disableDriveControl;
        this.enableDriveControl = enableDriveControl;
    }

    @Override
    public void initialize() {
        this.tx = txSupplier.get();
        this.px = pxSupplier.get();
        this.py = pySupplier.get();
        this.ty = tySupplier.get();
        isAdjusted = false;
        moveAction = null;
        disableDriveControl.run();
    }
    public double pixToAngle(double px) { // return rad
        return Math.atan((px-ConfigVariables.Camera.CAMERA_MATRIX[0][2]) / ConfigVariables.Camera.CAMERA_MATRIX[0][0]);
    }
    @Override
    public void execute(TelemetryPacket packet) {
        packet.put("DistanceAdjustLUTX/dx_input", tx);
        packet.put("DistanceAdjustLUTX/py_input", py);
        packet.put("DistanceAdjustLUTX/isAdjusted", isAdjusted);
        packet.put("DistanceAdjustLUTX/hasActiveAction", moveAction != null);

        // If we have an active movement action, run it
        if (moveAction != null) {
            // Check if action is done or timed out
            boolean actionDone = !moveAction.run(packet);
            lowSlide.updatePID();
            if (actionDone) {
                isAdjusted = true;
                moveAction = null;
                packet.put("DistanceAdjustLUTX/actionStatus", actionDone ? "completed" : "timed out");
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
            // final double gradient = lutratio.get(dy);
            // dx = dx - dy * gradient;
            // Δtx = 180/pi arctan(tan(ty*pi/180)*gradientpx))

//            final double gradientpx = ConfigVariables.Camera.XYPIXELRATIO;
//            final double pixelToAnglex = ConfigVariables.Camera.FOV[0]/ConfigVariables.Camera.RESOLUTION[0];
//            final double ddx = py*gradientpx*pixelToAnglex;
//
//            final double gradientpx = ConfigVariables.Camera.XYPIXELRATIO;
//            final double ddx = Math.toDegrees(Math.atan(Math.tan(Math.toRadians(ty)) * gradientpx));

            final double[] vanishingPoint = ConfigVariables.Camera.VANISHING_POINT;
            // find the line (px, py) to (vanishingPoint[0], vanishingPoint[1])
            final double m = (vanishingPoint[0] - px) / (vanishingPoint[1] - py);
            final double b = px - m * py;
            packet.put("vision/m", m);
            packet.put("vision/b", b);
            final double crosshairY = ConfigVariables.Camera.CROSSHAIR_Y_PX;
            // find x in px at crosshairY, x = my+b
            final double newpx = m*crosshairY + b;
            packet.put("vision/newpx", newpx);
            // tx = (px-cx)/fx
            final double fx1 = pixToAngle(newpx);
            final double fx2 = pixToAngle(ConfigVariables.Camera.CROSSHAIR_X_PX);
            final double newtx = Math.toDegrees(fx1 - fx2);
            packet.put("vision/newtx", newtx);
            tx = newtx;
            adjust(tx, ty, packet);
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

    public void adjust(double tx, double ty, TelemetryPacket packet) {
        double dxcm = lutx.get(tx);
        packet.put("vision/dxcm", dxcm);
        double dycm = luty.get(ty);
        packet.put("vision/dycm", dycm);
        double lowslideExtendcm = lowSlide.getCurrentPositionCM();
        // convert x,y to theta,r
        // theta = atan2(y,x)
        // r = sqrt(x^2 + y^2)
        double dyrobot = dycm + ConfigVariables.Camera.HALF_ROBOT_LENGTH + lowslideExtendcm;
        double theta = Math.atan2(-dxcm, dyrobot);
        double r = Math.sqrt(dxcm * dxcm + dyrobot * dyrobot) - ConfigVariables.Camera.HALF_ROBOT_LENGTH - lowslideExtendcm;
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
        // theta
        moveAction = drive.actionBuilder(startpose).turnTo(startpose.heading.toDouble() + theta).build();
        // Run the action first time
        drive.updatePoseEstimate();
        moveAction.run(packet);
    }
}
