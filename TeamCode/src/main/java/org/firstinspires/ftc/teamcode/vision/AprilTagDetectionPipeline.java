package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.utils.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.utils.constants.VisionConstants.Camera;
import org.firstinspires.ftc.teamcode.utils.constants.VisionConstants.AprilTag;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;

/**
 * Pipeline for detecting AprilTags in camera frames.
 * Uses tag positions from VisionConstants for pose estimation.
 */
public class AprilTagDetectionPipeline extends OpenCvPipeline {
        // Threading and management
        private long nativeApriltagPtr;
        private final Object detectionsUpdateSync = new Object();
        private ArrayList<AprilTagDetection> detections = new ArrayList<>();
        private Mat grey = new Mat();
        private boolean enableVisualization = true;

        // Visualization constants
        private static final Scalar BLUE = new Scalar(7, 197, 235, 255);
        private static final Scalar RED = new Scalar(255, 0, 0, 255);
        private static final Scalar GREEN = new Scalar(0, 255, 0, 255);
        private static final int TEXT_FACE = Imgproc.FONT_HERSHEY_SIMPLEX;
        private static final double TEXT_SCALE = 0.5;
        private static final int TEXT_THICKNESS = 2;

        public AprilTagDetectionPipeline() {
                // Initialize Apriltag native detector
                nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
                                AprilTagDetectorJNI.TagFamily.TAG_36h11.string,
                                AprilTag.MAX_HAMMING,
                                AprilTag.DECIMATION_HIGH);
        }

        @Override
        public Mat processFrame(Mat input) {
                // Convert to grayscale
                Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

                synchronized (detectionsUpdateSync) {
                        // Detect AprilTags
                        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(
                                        nativeApriltagPtr,
                                        grey,
                                        AprilTag.THRESHOLD_HIGH,
                                        AprilTag.THRESHOLD_LOW,
                                        AprilTag.DECIMATION_HIGH);
                }

                // Draw detections on input image if enabled
                if (enableVisualization) {
                        synchronized (detectionsUpdateSync) {
                                for (AprilTagDetection detection : detections) {
                                        drawDetection(input, detection);
                                        drawPoseInfo(input, detection);
                                }
                        }
                }

                return input;
        }

        private void drawDetection(Mat mat, AprilTagDetection detection) {
                // Draw center point
                Imgproc.circle(mat, new Point(detection.center.x, detection.center.y),
                                4, RED, 2);

                // Draw tag outline
                for (int i = 0; i <= 3; i++) {
                        int j = (i + 1) % 4;
                        Point pt1 = new Point(detection.corners[i].x, detection.corners[i].y);
                        Point pt2 = new Point(detection.corners[j].x, detection.corners[j].y);
                        Imgproc.line(mat, pt1, pt2, GREEN, 2);
                }
        }

        private void drawPoseInfo(Mat mat, AprilTagDetection detection) {
                VisionConstants.TagPosition tagPosition = VisionConstants.TAG_POSITIONS.get(detection.id);
                if (tagPosition == null)
                        return;

                // Calculate distance and angles
                double distance = Math.hypot(detection.pose.x, detection.pose.z);
                double yaw = Math.toDegrees(detection.pose.yaw);
                double pitch = Math.toDegrees(detection.pose.pitch);

                // Draw detection info
                ArrayList<String> info = new ArrayList<>();
                info.add(String.format("ID %d", detection.id));
                info.add(String.format("Dist: %.1f\"", distance * 39.3701)); // Convert meters to inches
                info.add(String.format("Yaw: %.1f°", yaw));
                info.add(String.format("Pitch: %.1f°", pitch));

                // Draw text lines
                Point textPos = new Point(detection.center.x - 50, detection.center.y - 50);
                for (String line : info) {
                        Imgproc.putText(mat, line, textPos, TEXT_FACE, TEXT_SCALE, BLUE, TEXT_THICKNESS);
                        textPos.y += 20;
                }
        }

        public void setVisualization(boolean enable) {
                enableVisualization = enable;
        }

        /**
         * Get the latest detections that meet confidence threshold
         */
        public ArrayList<AprilTagDetection> getLatestDetections() {
                ArrayList<AprilTagDetection> validDetections = new ArrayList<>();
                synchronized (detectionsUpdateSync) {
                        for (AprilTagDetection detection : detections) {
                                if (detection.decisionMargin > AprilTag.MIN_CONFIDENCE) {
                                        validDetections.add(detection);
                                }
                        }
                }
                return validDetections;
        }

        /**
         * Check if there are any valid detections
         */
        public boolean hasDetections() {
                synchronized (detectionsUpdateSync) {
                        for (AprilTagDetection detection : detections) {
                                if (detection.decisionMargin > AprilTag.MIN_CONFIDENCE) {
                                        return true;
                                }
                        }
                        return false;
                }
        }

        /**
         * Get estimated robot pose from tag detection
         */
        public Pose2d getEstimatedPose(AprilTagDetection detection) {
                VisionConstants.TagPosition tagPosition = VisionConstants.TAG_POSITIONS.get(detection.id);
                if (tagPosition == null)
                        return null;

                // Apply camera mount transform
                double x = detection.pose.x + Camera.MOUNT_X;
                double y = detection.pose.y + Camera.MOUNT_Y;
                double z = detection.pose.z + Camera.MOUNT_Z;

                // Calculate field position
                double fieldX = tagPosition.x + (x * Math.cos(tagPosition.rotationDegrees) -
                                y * Math.sin(tagPosition.rotationDegrees));
                double fieldY = tagPosition.y + (x * Math.sin(tagPosition.rotationDegrees) +
                                y * Math.cos(tagPosition.rotationDegrees));
                double fieldHeading = tagPosition.rotationDegrees + Math.toDegrees(detection.pose.yaw);

                return new Pose2d(fieldX, fieldY, Math.toRadians(fieldHeading));
        }

        @Override
        public void finalize() {
                // Free native resources
                if (nativeApriltagPtr != 0) {
                        AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
                        nativeApriltagPtr = 0;
                }
                grey.release();
        }
}
