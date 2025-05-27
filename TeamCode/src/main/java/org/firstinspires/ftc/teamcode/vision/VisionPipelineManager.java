package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Enhanced vision pipeline manager supporting multiple detection modes,
 * calibration, and detailed performance monitoring. Coordinates with
 * VisionStateManager for robust state-based pipeline management.
 */
public class VisionPipelineManager {
        private final OpenCvCamera camera;
        private final VisionStateManager stateManager;
        private final LoopTimer loopTimer;
        private final Telemetry telemetry;
        private final ElapsedTime pipelineTimer;
        private OpenCvPipeline currentPipeline;
        private VisionState lastState;

        // Pipeline performance tracking
        private long lastFrameTime = 0;
        private int frameCount = 0;
        private double avgProcessingTime = 0;
        private boolean isPipelineSwitching = false;

        // Camera configuration
        private static final int CAMERA_WIDTH = 640;
        private static final int CAMERA_HEIGHT = 480;
        private static final int CAMERA_FPS = 30;
        private static final int MIN_FRAME_MS = 1000 / CAMERA_FPS;

        public VisionPipelineManager(HardwareMap hardwareMap, VisionStateManager stateManager,
                        LoopTimer loopTimer, Telemetry telemetry) {
                this.stateManager = stateManager;
                this.loopTimer = loopTimer;
                this.telemetry = telemetry;
                this.pipelineTimer = new ElapsedTime();
                this.lastState = VisionState.INITIALIZING;

                // Initialize camera with error handling
                try {
                        int cameraMonitorViewId = ControlHub.getCameraMonitorViewId(hardwareMap);
                        camera = OpenCvCameraFactory.getInstance().createWebcam(
                                        ControlHub.getWebcamName(hardwareMap), cameraMonitorViewId);

                        configureCameraSettings();
                        initializeCamera();
                } catch (Exception e) {
                        telemetry.addData("Camera Init Error", e.getMessage());
                        telemetry.update();
                        throw new RuntimeException("Camera initialization failed", e);
                }
        }

        private void configureCameraSettings() {
                camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                camera.setPipeline(new CalibrationPipeline());
        }

        private void initializeCamera() {
                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                        @Override
                        public void onOpened() {
                                try {
                                        camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT,
                                                        OpenCvCameraRotation.UPRIGHT);
                                        telemetry.addData("Camera", "Initialized");
                                        telemetry.update();
                                        stateManager.setCalibrated(true);
                                } catch (Exception e) {
                                        handleCameraError("Stream start failed: " + e.getMessage());
                                }
                        }

                        @Override
                        public void onError(int errorCode) {
                                handleCameraError("Error code: " + errorCode);
                        }
                });
        }

        /**
         * Update pipeline based on current vision state
         */
        public void update() {
                loopTimer.startSection("vision_pipeline");
                pipelineTimer.reset();

                try {
                        VisionState currentState = stateManager.getCurrentState();
                        if (shouldSwitchPipeline(currentState)) {
                                switchPipeline(currentState);
                        }
                        lastState = currentState;

                        updatePerformanceMetrics();
                } catch (Exception e) {
                        handlePipelineError(e);
                } finally {
                        loopTimer.endSection("vision_pipeline");
                }
        }

        private boolean shouldSwitchPipeline(VisionState currentState) {
                return !isPipelineSwitching &&
                                currentState != lastState &&
                                currentState.requiresPipelineSwitch();
        }

        private void switchPipeline(VisionState newState) {
                isPipelineSwitching = true;
                try {
                        OpenCvPipeline newPipeline = createPipeline(newState);
                        if (newPipeline != null && currentPipeline != newPipeline) {
                                camera.setPipeline(newPipeline);
                                currentPipeline = newPipeline;
                                resetPerformanceMetrics();
                        }
                } catch (Exception e) {
                        handlePipelineError(e);
                } finally {
                        isPipelineSwitching = false;
                }
        }

        private OpenCvPipeline createPipeline(VisionState state) {
                switch (state) {
                        case CALIBRATING:
                                return new CalibrationPipeline();
                        case SEARCHING:
                        case DETECTING:
                                return new DetectionPipeline();
                        case TRACKING_SINGLE:
                                return new SingleTrackingPipeline();
                        case TRACKING_MULTIPLE:
                                return new MultiTrackingPipeline();
                        case INITIALIZING:
                                return new InitializationPipeline();
                        default:
                                return null;
                }
        }

        private void updatePerformanceMetrics() {
                frameCount++;
                long currentTime = System.currentTimeMillis();
                if (lastFrameTime > 0) {
                        long frameTime = currentTime - lastFrameTime;
                        avgProcessingTime = (avgProcessingTime * (frameCount - 1) + frameTime) / frameCount;
                }
                lastFrameTime = currentTime;
        }

        private void resetPerformanceMetrics() {
                frameCount = 0;
                avgProcessingTime = 0;
                lastFrameTime = 0;
        }

        private void handleCameraError(String message) {
                telemetry.addData("Camera Error", message);
                telemetry.update();
                stateManager.update(new ArrayList<>()); // Empty detection list to trigger error state
        }

        private void handlePipelineError(Exception e) {
                telemetry.addData("Pipeline Error", e.getMessage());
                telemetry.update();
                stateManager.update(new ArrayList<>()); // Empty detection list to trigger error state
        }

        /**
         * Get current performance metrics
         */
        public String getPerformanceMetrics() {
                return String.format("FPS: %.1f, Avg Process: %.1fms, Frames: %d",
                                1000.0 / Math.max(avgProcessingTime, MIN_FRAME_MS),
                                avgProcessingTime,
                                frameCount);
        }

        /**
         * Stop camera streaming and release resources
         */
        public void shutdown() {
                try {
                        camera.stopStreaming();
                        camera.closeCameraDevice();
                } catch (Exception e) {
                        telemetry.addData("Shutdown Error", e.getMessage());
                        telemetry.update();
                }
        }

        // Enhanced base pipeline with common functionality
        private abstract class BasePipeline extends OpenCvPipeline {
                protected final List<VisionStateManager.Detection> detections = new ArrayList<>();
                protected final Mat processedFrame = new Mat();
                protected final ElapsedTime frameTimer = new ElapsedTime();

                @Override
                public Mat processFrame(Mat input) {
                        frameTimer.reset();
                        detections.clear();
                        input.copyTo(processedFrame);

                        try {
                                processFrameInternal(processedFrame);
                                drawDetections(processedFrame);
                                reportMetrics();
                        } catch (Exception e) {
                                handlePipelineError(e);
                        }

                        return processedFrame;
                }

                protected abstract void processFrameInternal(Mat frame);

                protected void drawDetections(Mat frame) {
                        for (VisionStateManager.Detection detection : detections) {
                                // Draw detection visualization
                                Imgproc.circle(frame, new Point(frame.cols() / 2, frame.rows() / 2),
                                                5, new Scalar(0, 255, 0), 2);
                        }
                }

                protected void reportMetrics() {
                        stateManager.update(detections);
                }
        }

        // Specialized pipeline implementations
        private class CalibrationPipeline extends BasePipeline {
                @Override
                protected void processFrameInternal(Mat frame) {
                        // Implement camera calibration
                        if (performCalibration(frame)) {
                                detections.add(new VisionStateManager.Detection(1.0, null));
                        }
                }

                private boolean performCalibration(Mat frame) {
                        // Add calibration logic here
                        return true; // Return true when calibration successful
                }
        }

        private class DetectionPipeline extends BasePipeline {
                @Override
                protected void processFrameInternal(Mat frame) {
                        // Implement detection pipeline
                        // Example: detect objects and add to detections list
                        detectObjects(frame).forEach(obj -> detections.add(new VisionStateManager.Detection(
                                        calculateConfidence(obj), obj)));
                }

                private List<Object> detectObjects(Mat frame) {
                        // Add object detection logic here
                        return new ArrayList<>();
                }

                private double calculateConfidence(Object detection) {
                        // Add confidence calculation logic here
                        return 0.0;
                }
        }

        private class SingleTrackingPipeline extends BasePipeline {
                private Object lastTarget = null;

                @Override
                protected void processFrameInternal(Mat frame) {
                        if (lastTarget != null) {
                                Object updatedTarget = trackTarget(frame, lastTarget);
                                if (updatedTarget != null) {
                                        detections.add(new VisionStateManager.Detection(
                                                        calculateTrackingConfidence(updatedTarget),
                                                        updatedTarget));
                                }
                        }
                }

                private Object trackTarget(Mat frame, Object target) {
                        // Add single target tracking logic here
                        return null;
                }

                private double calculateTrackingConfidence(Object target) {
                        // Add tracking confidence calculation logic here
                        return 0.0;
                }
        }

        private class MultiTrackingPipeline extends BasePipeline {
                private List<Object> lastTargets = new ArrayList<>();

                @Override
                protected void processFrameInternal(Mat frame) {
                        List<Object> currentTargets = trackMultipleTargets(frame, lastTargets);
                        for (Object target : currentTargets) {
                                detections.add(new VisionStateManager.Detection(
                                                calculateTrackingConfidence(target),
                                                target));
                        }
                        lastTargets = currentTargets;
                }

                private List<Object> trackMultipleTargets(Mat frame, List<Object> targets) {
                        // Add multiple target tracking logic here
                        return new ArrayList<>();
                }

                private double calculateTrackingConfidence(Object target) {
                        // Add multi-target tracking confidence calculation here
                        return 0.0;
                }
        }
}
