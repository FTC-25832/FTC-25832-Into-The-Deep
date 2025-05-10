package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.opencv.core.Point;
import java.util.ArrayList;
import java.util.List;

public class Localizer {
        /*
         * Odometry
         */
        public static DcMotor podL, podR, podM; // pod left, pod right, pod middle.
        HardwareMap hardwareMap;
        public static double C = (35 * 3.14) / 8192;
        // positive x should be the forward direction of the robot
        // positive y should be left
        // in milimeters
        public static double Ly = 126.59; // distance between left L wheel and Y
        public static double Ry = -135.73; // distance between right R wheel and Y
        public static double Bx = 42.50; // distance between center C wheel and X, B as in 'back'

        public static double X = 0;
        public static double Y = 0;
        public static double theta = 0;

        public static long previousLeftEncoderPos, previousRightEncoderPos, previousMiddleEncoderPos, leftEncoderPos,
                        rightEncoderPos, middleEncoderPos;
        public static double deltaLeftEncoderPos, deltaRightEncoderPos, deltaMiddleEncoderPos;

        public void initialize(HardwareMap map, String podLname, String podRname, String podMname) {
                hardwareMap = map;
                List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
                for (LynxModule hub : allHubs) {
                        hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                }

                podR = hardwareMap.get(DcMotor.class, podRname);
                podL = hardwareMap.get(DcMotor.class, podLname);
                podM = hardwareMap.get(DcMotor.class, podMname);
        }

        public void initialize(HardwareMap map) {
                initialize(map, ControlHub.motor(0), ControlHub.motor(2), ControlHub.motor(3));
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
                return theta;
        }

        public static void positionArc() {
                double dX, dY;
                leftEncoderPos = -podL.getCurrentPosition(); // negative
                rightEncoderPos = -podR.getCurrentPosition();
                middleEncoderPos = podM.getCurrentPosition();

                deltaLeftEncoderPos = leftEncoderPos - previousLeftEncoderPos;
                deltaRightEncoderPos = rightEncoderPos - previousRightEncoderPos;
                deltaMiddleEncoderPos = middleEncoderPos - previousMiddleEncoderPos;

                deltaLeftEncoderPos *= C;
                deltaRightEncoderPos *= C;
                deltaMiddleEncoderPos *= C;

                // double phi = (deltaLeftEncoderPos-deltaRightEncoderPos) / (Ly-Ry);
                // double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
                // double deltaPerpPos = deltaMiddleEncoderPos - Bx * phi;

                // xPos += dX;
                // yPos += dY;
                // heading += phi;

                previousMiddleEncoderPos = middleEncoderPos;
                previousLeftEncoderPos = leftEncoderPos;
                previousRightEncoderPos = rightEncoderPos;

                double fwd = (deltaRightEncoderPos * Ly - deltaLeftEncoderPos * Ry) / (Ly - Ry); // change in relative x
                                                                                                 // direction
                double dA = (deltaRightEncoderPos - deltaLeftEncoderPos) / (Ly - Ry); // theta, the heading, delta angle
                double str = deltaMiddleEncoderPos - Bx * dA; // strafe
                // bx is the x position of the middle odometer
                // ly is the y position of the left odometer...

                // rX = relative x
                if (dA != 0) {
                        double r0 = fwd / dA;
                        double r1 = str / dA;
                        // relative x and y
                        double relX = r0 * Math.sin(dA) - r1 * (1 - Math.cos(dA));
                        double relY = r1 * Math.sin(dA) + r0 * (1 - Math.cos(dA));
                        dX = relX * Math.cos(theta) - relY * Math.sin(theta);
                        dY = relY * Math.cos(theta) + relX * Math.sin(theta);
                } else {
                        dX = fwd * Math.cos(theta) - str * Math.sin(theta);
                        dY = str * Math.cos(theta) + fwd * Math.sin(theta);
                }
                // upd x and upd y, ipdf theta

                X += dX;
                Y += dY;
                theta += dA;
        }

        public static double angleWrap(double radians) {

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

                double quadC = (Math.pow(m1, 2) * Math.pow(x1, 2) - (2 * y1 * m1 * x1) + Math.pow(y1, 2)
                                - Math.pow(r, 2));

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
