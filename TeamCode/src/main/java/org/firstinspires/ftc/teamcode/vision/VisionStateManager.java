package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;

import java.util.ArrayList;
import java.util.List;

/**
 * Enhanced vision state manager with support for multiple detection tracking,
 * calibration, and detailed error handling.
 */
public class VisionStateManager implements VisionStateContext {
        private final ElapsedTime stateTimer = new ElapsedTime();
        private final ElapsedTime processingTimer = new ElapsedTime();
        private final LoopTimer loopTimer;
        private final List<Double> currentConfidences = new ArrayList<>();

        // State tracking
        private VisionState currentState = VisionState.INITIALIZING;
        private boolean hasCurrentDetection = false;
        private boolean isCalibrated = false;
        private boolean needsCalibration = true;
        private int detectionCount = 0;
        private double currentConfidence = 0.0;

        // Error handling
        private int retryCount = 0;
        private static final int MAX_RETRIES = 3;
        private boolean pipelineOperational = true;
        private boolean cameraOperational = true;
        private long lastProcessingTime = 0;

        // Vision system references
        private VuforiaLocalizer vuforia = null;
        private boolean isVuforiaInitialized = false;

        public VisionStateManager(LoopTimer loopTimer) {
                this.loopTimer = loopTimer;
                stateTimer.reset();
                processingTimer.reset();
        }

        /**
         * Update vision state with new detection data
         */
        public void update(List<Detection> detections) {
                loopTimer.startSection("vision_state");
                processingTimer.reset();

                try {
                        updateDetectionData(detections);
                        VisionState nextState = currentState.nextState(this);

                        if (nextState != currentState) {
                                handleStateTransition(nextState);
                        }

                        lastProcessingTime = (long) processingTimer.milliseconds();
                } catch (Exception e) {
                        pipelineOperational = false;
                        handleError(e);
                }

                loopTimer.endSection("vision_state");
        }

        private void updateDetectionData(List<Detection> detections) {
                currentConfidences.clear();
                detectionCount = detections.size();
                hasCurrentDetection = detectionCount > 0;

                if (hasCurrentDetection) {
                        // Update confidence values
                        for (Detection detection : detections) {
                                currentConfidences.add(detection.confidence);
                        }
                        currentConfidence = detections.get(0).confidence;
                } else {
                        currentConfidence = 0.0;
                }
        }

        private void handleStateTransition(VisionState newState) {
                loopTimer.startSection("state_transition");

                // Reset timer on state change
                stateTimer.reset();

                // Handle retry counting
                if (currentState.isErrorState() && newState == VisionState.INITIALIZING) {
                        retryCount++;
                } else if (newState.isErrorState()) {
                        retryCount = 0;
                }

                // Special state transition handling
                if (newState.requiresPipelineSwitch()) {
                        handlePipelineSwitch(newState);
                }

                // Update state
                currentState = newState;
                loopTimer.endSection("state_transition");
        }

        private void handlePipelineSwitch(VisionState newState) {
                try {
                        Thread.sleep(newState.getTransitionDelay());
                        // Additional pipeline switch logic could go here
                } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        pipelineOperational = false;
                }
        }

        private void handleError(Exception e) {
                currentState = getErrorState();
                if (currentState == VisionState.ERROR_CAMERA) {
                        cameraOperational = false;
                }
        }

        // VisionStateContext implementation
        @Override
        public boolean isInitialized() {
                return isVuforiaInitialized && stateTimer.milliseconds() > 500;
        }

        @Override
        public boolean needsCalibration() {
                return needsCalibration;
        }

        @Override
        public boolean isCalibrated() {
                return isCalibrated;
        }

        @Override
        public boolean hasDetection() {
                return hasCurrentDetection;
        }

        @Override
        public boolean hasMultipleDetections() {
                return detectionCount > 1;
        }

        @Override
        public int getDetectionCount() {
                return detectionCount;
        }

        @Override
        public boolean isTimeout() {
                return stateTimer.milliseconds() >= currentState.getTimeout();
        }

        @Override
        public boolean canRetry() {
                return retryCount < MAX_RETRIES;
        }

        @Override
        public long getStateTime() {
                return (long) stateTimer.milliseconds();
        }

        @Override
        public double getConfidence() {
                return currentConfidence;
        }

        @Override
        public List<Double> getMultipleConfidences() {
                return new ArrayList<>(currentConfidences);
        }

        @Override
        public boolean isPipelineOperational() {
                return pipelineOperational;
        }

        @Override
        public boolean isCameraOperational() {
                return cameraOperational;
        }

        @Override
        public long getProcessingTime() {
                return lastProcessingTime;
        }

        /**
         * Get current vision state
         */
        public VisionState getCurrentState() {
                return currentState;
        }

        /**
         * Check if actively processing
         */
        public boolean isProcessing() {
                return currentState.isProcessingActive();
        }

        /**
         * Get retry count
         */
        public int getRetryCount() {
                return retryCount;
        }

        /**
         * Initialize Vuforia
         */
        public void initializeVuforia(VuforiaLocalizer vuforia) {
                this.vuforia = vuforia;
                this.isVuforiaInitialized = true;
        }

        /**
         * Set calibration status
         */
        public void setCalibrated(boolean calibrated) {
                this.isCalibrated = calibrated;
                this.needsCalibration = !calibrated;
        }

        /**
         * Reset manager state
         */
        public void reset() {
                currentState = VisionState.INITIALIZING;
                hasCurrentDetection = false;
                currentConfidence = 0.0;
                retryCount = 0;
                detectionCount = 0;
                currentConfidences.clear();
                stateTimer.reset();
                processingTimer.reset();
                pipelineOperational = true;
                cameraOperational = true;
                lastProcessingTime = 0;
        }

        /**
         * Data class for vision detections
         */
        public static class Detection {
                public final double confidence;
                public final Object data; // Could be customized based on detection type

                public Detection(double confidence, Object data) {
                        this.confidence = confidence;
                        this.data = data;
                }
        }
}
