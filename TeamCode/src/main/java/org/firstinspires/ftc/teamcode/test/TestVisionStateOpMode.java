package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.vision.VisionPipelineManager;
import org.firstinspires.ftc.teamcode.vision.VisionState;
import java.util.ArrayList;
import java.util.List;

/**
 * Enhanced test OpMode for validating vision system functionality.
 * Tests multiple detection tracking, calibration, pipeline switching,
 * and performance monitoring. Uses gamepad inputs for comprehensive testing.
 */
@TeleOp(name = "Test: Vision System", group = "Test")
public class TestVisionStateOpMode extends BaseOpMode {
        private VisionStateManager stateManager;
        private VisionPipelineManager pipelineManager;

        // Button state tracking
        private boolean lastAState = false;
        private boolean lastBState = false;
        private boolean lastXState = false;
        private boolean lastYState = false;
        private boolean lastDpadUpState = false;
        private boolean lastDpadDownState = false;
        private boolean lastRightBumper = false;

        // Simulated detection data
        private final List<VisionStateManager.Detection> detections = new ArrayList<>();
        private double simulatedConfidence = 0.8;
        private int detectionCount = 1;
        private boolean isCalibrated = false;

        // Performance simulation
        private double simulatedProcessingTime = 30.0;
        private boolean showDetailedMetrics = false;

        @Override
        protected void initialize() {
                stateManager = new VisionStateManager(loopTimer);
                pipelineManager = new VisionPipelineManager(hardwareMap, stateManager, loopTimer, telemetry);

                displayControls();
                initializeDetections();
        }

        private void displayControls() {
                telemetry.addLine("=== Vision Test Controls ===");
                telemetry.addData("A Button", "Toggle Detection");
                telemetry.addData("B Button", "Simulate Error");
                telemetry.addData("X Button", "Toggle Detailed Metrics");
                telemetry.addData("Y Button", "Reset System");
                telemetry.addData("DPad Up/Down", "Add/Remove Detection");
                telemetry.addData("Left Stick Y", "Confidence (0-1)");
                telemetry.addData("Right Stick Y", "Processing Time");
                telemetry.addData("Right Bumper", "Toggle Calibration");
                telemetry.update();
        }

        private void initializeDetections() {
                detections.clear();
                detections.add(new VisionStateManager.Detection(simulatedConfidence, new Object()));
        }

        @Override
        protected void update() {
                pipelineManager.update();
                handleGamepadInput();
                updateDetections();

                // Update state manager with current detections
                if (stateManager.isProcessing()) {
                        stateManager.update(detections);
                }
        }

        private void handleGamepadInput() {
                // Toggle detection with A button
                boolean currentAState = gamepad1.a;
                if (currentAState && !lastAState) {
                        detectionCount = detectionCount > 0 ? 0 : 1;
                        updateDetections();
                }
                lastAState = currentAState;

                // Simulate errors with B button
                boolean currentBState = gamepad1.b;
                if (currentBState && !lastBState) {
                        simulateError();
                }
                lastBState = currentBState;

                // Toggle detailed metrics with X button
                boolean currentXState = gamepad1.x;
                if (currentXState && !lastXState) {
                        showDetailedMetrics = !showDetailedMetrics;
                }
                lastXState = currentXState;

                // Reset system with Y button
                boolean currentYState = gamepad1.y;
                if (currentYState && !lastYState) {
                        resetSystem();
                }
                lastYState = currentYState;

                // Add/remove detections with DPad
                boolean currentDpadUpState = gamepad1.dpad_up;
                if (currentDpadUpState && !lastDpadUpState) {
                        detectionCount = Math.min(detectionCount + 1, 3);
                        updateDetections();
                }
                lastDpadUpState = currentDpadUpState;

                boolean currentDpadDownState = gamepad1.dpad_down;
                if (currentDpadDownState && !lastDpadDownState) {
                        detectionCount = Math.max(detectionCount - 1, 0);
                        updateDetections();
                }
                lastDpadDownState = currentDpadDownState;

                // Toggle calibration with right bumper
                boolean currentRightBumper = gamepad1.right_bumper;
                if (currentRightBumper && !lastRightBumper) {
                        isCalibrated = !isCalibrated;
                        stateManager.setCalibrated(isCalibrated);
                }
                lastRightBumper = currentRightBumper;

                // Update confidence with left stick
                double rawConfidence = (-gamepad1.left_stick_y + 1.0) / 2.0;
                simulatedConfidence = Math.min(Math.max(rawConfidence, 0.0), 1.0);

                // Update processing time with right stick
                double rawTime = (-gamepad1.right_stick_y + 1.0) / 2.0;
                simulatedProcessingTime = rawTime * 100; // 0-100ms range
        }

        private void updateDetections() {
                detections.clear();
                for (int i = 0; i < detectionCount; i++) {
                        detections.add(new VisionStateManager.Detection(simulatedConfidence, new Object()));
                }
        }

        private void simulateError() {
                if (Math.random() < 0.5) {
                        // Simulate pipeline error
                        stateManager.update(new ArrayList<>());
                } else {
                        // Simulate camera error
                        pipelineManager.handleCameraError("Simulated camera failure");
                }
        }

        private void resetSystem() {
                stateManager.reset();
                pipelineManager.update();
                initializeDetections();
                isCalibrated = false;
                simulatedConfidence = 0.8;
                simulatedProcessingTime = 30.0;
                detectionCount = 1;
        }

        @Override
        protected void addCustomTelemetry() {
                telemetry.addLine("=== Vision System Status ===");
                telemetry.addData("State", stateManager.getCurrentState());
                telemetry.addData("Detections", String.format("%d @ %.2f conf", detectionCount, simulatedConfidence));
                telemetry.addData("Calibrated", isCalibrated);
                telemetry.addData("Processing", stateManager.isProcessing());

                VisionState state = stateManager.getCurrentState();
                if (showDetailedMetrics) {
                        telemetry.addLine("\n=== Performance Metrics ===");
                        telemetry.addData("Processing Time", String.format("%.1f ms", simulatedProcessingTime));
                        telemetry.addData("Frame Stats", pipelineManager.getPerformanceMetrics());
                        telemetry.addData("State Time", String.format("%d ms", stateManager.getStateTime()));
                        telemetry.addData("Retries", stateManager.getRetryCount());

                        telemetry.addLine("\n=== Pipeline Details ===");
                        telemetry.addData("Pipeline Switch", state.requiresPipelineSwitch());
                        telemetry.addData("High Confidence", state.requiresHighConfidence());
                        telemetry.addData("Error State", state.isErrorState());
                        telemetry.addData("Tracking Mode", state.isTrackingState());
                }

                telemetry.addLine("\n=== Instructions ===");
                telemetry.addData("Press X", "Toggle Detailed Stats");
        }

        @Override
        public void stop() {
                try {
                        if (pipelineManager != null) {
                                pipelineManager.shutdown();
                        }
                        if (stateManager != null) {
                                stateManager.reset();
                        }
                } catch (Exception e) {
                        telemetry.addData("Shutdown Error", e.getMessage());
                        telemetry.update();
                }
                super.stop();
        }
}
