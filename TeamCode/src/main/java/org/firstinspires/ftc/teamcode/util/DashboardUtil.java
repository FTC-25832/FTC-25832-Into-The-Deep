package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.List;

/**
 * Set of helper functions for drawing FTC Dashboard visualizations.
 */
public class DashboardUtil {
        private static final double ROBOT_RADIUS = 230; // mm (9 inches)
        private static final double POLE_RADIUS = 12.7; // mm (0.5 inches)

        /**
         * localizer data
         */
        public static void drawRobot(Canvas canvas, String color) {
                // Convert from mm to inches for display
                double xInches = Localizer.X / 25.4;
                double yInches = Localizer.Y / 25.4;
                double theta = Localizer.theta; // theta is already in radians

                canvas.setStroke(color);
                canvas.strokeCircle(xInches, yInches, ROBOT_RADIUS / 25.4);
                canvas.strokeCircle(xInches, yInches, 12.7 / 25.4); // 0.5 inch center point

                // Draw heading indicator line
                double headingX = xInches + Math.cos(theta) * (ROBOT_RADIUS / 25.4);
                double headingY = yInches + Math.sin(theta) * (ROBOT_RADIUS / 25.4);
                canvas.strokeLine(xInches, yInches, headingX, headingY);
        }

        /**
         * no heading
         */
        public static void drawRobotPosition(Canvas canvas) {
                double xInches = Localizer.X / 25.4;
                double yInches = Localizer.Y / 25.4;
                canvas.strokeCircle(xInches, yInches, ROBOT_RADIUS / 25.4);
        }

        /**
         * Draws a trail of robot poses from position history
         */
        public static void drawPoseHistory(Canvas canvas, List<double[]> poseHistory) {
                if (poseHistory.isEmpty())
                        return;

                double[] xPoints = new double[poseHistory.size()];
                double[] yPoints = new double[poseHistory.size()];

                // Convert from mm to inches for each point
                for (int i = 0; i < poseHistory.size(); i++) {
                        double[] pose = poseHistory.get(i);
                        xPoints[i] = pose[0] / 25.4; // X in inches
                        yPoints[i] = pose[1] / 25.4; // Y in inches
                }
                canvas.strokePolyline(xPoints, yPoints);
        }

        /**
         * target pos
         */
        public static void drawPole(Canvas canvas, double x, double y) {
                // Convert from mm to inches for display
                canvas.strokeCircle(x / 25.4, y / 25.4, POLE_RADIUS / 25.4);
        }

        /**
         * anglewrap to match localizer
         */
        public static double angleWrap(double radians) {
                return Localizer.angleWrap(radians);
        }
}
