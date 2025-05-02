package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.List;

public class Localizer {
    public static DcMotor podL, podR, podM;
    HardwareMap hardwareMap;
    public static double C = (35 * 3.14) / 8192; // (r*pi)/N(encoder counts for 1 loop)
    public static double Ly = 42; // distance between L wheels and y mm
    public static double Ry = -34; // distance between R wheels and y
    public static double Bx = 4.12; // distance between center wheel and x
    // initial values
    public static double X = 0;
    public static double Y = 0;
    public static double theta = 0;

    public static long pL, pR, pM, L, R, M;
    public static double dL, dR, dM;

    IMU imu;

    public void initialize(HardwareMap map, String podLname, String podRname, String podMname) {
        hardwareMap = map;
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        podR = hardwareMap.get(DcMotor.class, podRname);
        podL = hardwareMap.get(DcMotor.class, podLname);
        podM = hardwareMap.get(DcMotor.class, podMname);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void initialize(HardwareMap map) {
        initialize(map, expansion.motor(1), expansion.motor(0), expansion.motor(2));
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void resetEncoder() {
        X = 0;
        Y = 0;
        theta = 0;
        podL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        podR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        podM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        podL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        podR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        podM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double heading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public static void positionArc() {
        double dX, dY;
        L = -podL.getCurrentPosition();
        R = -podR.getCurrentPosition();
        M = podM.getCurrentPosition();
        // encoder deltas
        dL = (L - pL) * C;
        dR = (R - pR) * C;
        dM = (M - pM) * C;
        // update previous readings
        pL = L;
        pR = R;
        pM = M;
        // Calculate
        double fwd = (dR * Ly - dL * Ry) / (Ly - Ry); // forward
        double dA = (dR - dL) / (Ly - Ry); // angular
        double str = dM - Bx * dA; // strafe

        if (dA != 0) { // arc based calculation
            double r0 = fwd / dA;
            double r1 = str / dA;
            double rX = r0 * Math.sin(dA) - r1 * (1 - Math.cos(dA));
            double rY = r1 * Math.sin(dA) + r0 * (1 - Math.cos(dA));

            dX = rX * Math.cos(theta) - rY * Math.sin(theta);
            dY = rX * Math.sin(theta) + rY * Math.cos(theta);
        } else { // straight
            dX = fwd * Math.cos(theta) - str * Math.sin(theta);
            dY = fwd * Math.sin(theta) + str * Math.cos(theta);
        }
        // update current position
        X += dX;
        Y += dY;
        theta += dA;
        // theta = angleWrap(theta);
    }

    public static double angleWrap(double radians) {
        // keep radian within -PI to PI
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public static ArrayList<Point> intersect(Point center, double r, Point line1, Point line2) {
        // find intersection of circle center-r to line line1-line2
        ArrayList<Point> all = new ArrayList<>();
        if (line2.x - line1.x == 0) {
            double y1 = Math.sqrt(Math.pow(r, 2) - Math.pow(line1.x - center.x, 2));
            double y2 = -Math.sqrt(Math.pow(r, 2) - Math.pow(line1.x - center.x, 2));
            y1 += center.y;
            y2 += center.y;
            all.add(new Point(line1.x, y1));
            all.add(new Point(line1.x, y2));
            return all;
        }

        double m1 = (line2.y - line1.y) / (line2.x - line1.x);
        double quadA = 1 + Math.pow(m1, 2);

        double x1 = line1.x - center.x;
        double y1 = line1.y - center.y;

        double quadB = (2 * m1 * y1) - (2 * Math.pow(m1, 2) * x1);

        double quadC = (Math.pow(m1, 2) * Math.pow(x1, 2) - (2 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(r, 2));

        try {
            double delta = Math.sqrt(Math.pow(quadB, 2) - 4 * quadA * quadC);
            double xRoot1 = (-quadB + delta) / (2 * quadA);
            double xRoot2 = (-quadB - delta) / (2 * quadA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot1 += center.x;
            yRoot1 += center.y;
            xRoot2 += center.x;
            yRoot2 += center.y;

            double minX = Math.min(line1.x, line2.x);
            double maxX = Math.max(line1.x, line2.x);

            if (xRoot1 > minX && xRoot1 < maxX) {
                all.add(new Point(xRoot1, yRoot1));
            }
            if (xRoot2 > minX && xRoot2 < maxX) {
                all.add(new Point(xRoot2, yRoot2));
            }

        } catch (Exception e) {

        }
        return all;
    }
}
