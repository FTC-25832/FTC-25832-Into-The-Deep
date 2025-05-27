package org.firstinspires.ftc.teamcode.utils.visualization;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Path;

import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.vision.VisionState;
import com.example.meepmeeptesting.shared.RobotState;
import com.example.meepmeeptesting.shared.MovementConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * Real-time robot visualization utility using FTC Dashboard.
 * Provides visualization for:
 * - Current robot position and heading
 * - Planned paths and trajectories
 * - Vision system detections
 * - Robot state transitions
 */
public class RobotVisualizer {
        private final FtcDashboard dashboard;
        private final List<PathSegment> pathHistory;
        private Pose2d currentPose;
        private RobotState targetState;
        private VisionState visionState;
        private boolean hasDetection;

        // Drawing constants
        private static final double ROBOT_RADIUS = MovementConstants.RobotDimensions.BOT_WIDTH / 2.0;
        private static final double PATH_STROKE_WIDTH = 1.0;
        private static final double DETECTION_RADIUS = 3.0;

        public RobotVisualizer() {
                dashboard = FtcDashboard.getInstance();
                pathHistory = new ArrayList<>();
                currentPose = new Pose2d();
        }

        /**
         * Update robot position and state information
         */
        public void updateState(Pose2d pose, RobotState target, VisionState vision, boolean detected) {
                currentPose = pose;
                targetState = target;
                visionState = vision;
                hasDetection = detected;

                // Record path segment
                if (!pathHistory.isEmpty()) {
                        PathSegment lastSegment = pathHistory.get(pathHistory.size() - 1);
                        if (lastSegment.end.position.distTo(pose.position) > 1.0) {
                                pathHistory.add(new PathSegment(lastSegment.end, pose));
                        }
                } else {
                        pathHistory.add(new PathSegment(pose, pose));
                }

                // Limit path history size
                while (pathHistory.size() > 100) {
                        pathHistory.remove(0);
                }

                drawVisualization();
        }

        /**
         * Draw current robot state to dashboard
         */
        private void drawVisualization() {
                Canvas canvas = new Canvas();

                // Draw field elements
                drawField(canvas);

                // Draw path history
                drawPathHistory(canvas);

                // Draw current robot
                drawRobot(canvas, currentPose, "blue");

                // Draw target state if exists
                if (targetState != null) {
                        drawRobot(canvas, targetState.getPose(), "green");
                }

                // Draw vision detection
                if (hasDetection && visionState.isTrackingState()) {
                        Vector2d detectionPos = currentPose.position.plus(
                                        new Vector2d(ROBOT_RADIUS * 2, 0).rotated(currentPose.heading));
                        drawDetection(canvas, detectionPos);
                }

                // Draw to dashboard
                dashboard.sendTelemetryPacket(DashboardUtil.makePacket(canvas));
        }

        /**
         * Draw field elements and boundaries
         */
        private void drawField(Canvas canvas) {
                // Draw field border
                canvas.strokeRect(0, 0, 144, 144);

                // Draw navigation targets
                for (RobotState state : RobotState.values()) {
                        canvas.strokeCircle(state.getX(), state.getY(), 2);
                }
        }

        /**
         * Draw robot path history
         */
        private void drawPathHistory(Canvas canvas) {
                for (PathSegment segment : pathHistory) {
                        canvas.strokeLine(
                                        segment.start.position.x, segment.start.position.y,
                                        segment.end.position.x, segment.end.position.y);
                }
        }

        /**
         * Draw robot at specified pose
         */
        private void drawRobot(Canvas canvas, Pose2d pose, String color) {
                canvas.setStroke(color);

                // Draw robot body
                canvas.strokeCircle(pose.position.x, pose.position.y, ROBOT_RADIUS);

                // Draw heading indicator
                Vector2d heading = new Vector2d(ROBOT_RADIUS, 0).rotated(pose.heading);
                canvas.strokeLine(
                                pose.position.x, pose.position.y,
                                pose.position.x + heading.x, pose.position.y + heading.y);
        }

        /**
         * Draw vision system detection
         */
        private void drawDetection(Canvas canvas, Vector2d position) {
                canvas.setStroke("yellow");
                canvas.strokeCircle(position.x, position.y, DETECTION_RADIUS);
                canvas.strokeCircle(position.x, position.y, DETECTION_RADIUS * 0.5);
        }

        /**
         * Clear path history
         */
        public void reset() {
                pathHistory.clear();
                currentPose = new Pose2d();
                targetState = null;
                hasDetection = false;
        }

        /**
         * Path segment for recording robot movement
         */
        private static class PathSegment {
                public final Pose2d start;
                public final Pose2d end;

                public PathSegment(Pose2d start, Pose2d end) {
                        this.start = start;
                        this.end = end;
                }
        }
}
