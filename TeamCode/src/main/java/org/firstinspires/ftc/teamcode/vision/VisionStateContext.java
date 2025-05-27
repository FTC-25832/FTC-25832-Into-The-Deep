package org.firstinspires.ftc.teamcode.vision;

/**
 * Enhanced interface defining the context for vision state transitions.
 * Provides comprehensive information for state machine decision making,
 * including calibration, multiple detection tracking, and detailed error
 * handling.
 */
public interface VisionStateContext {
        /**
         * Check if camera and vision system are initialized
         */
        boolean isInitialized();

        /**
         * Check if system requires calibration
         */
        boolean needsCalibration();

        /**
         * Check if calibration is complete
         */
        boolean isCalibrated();

        /**
         * Check for valid target detection
         */
        boolean hasDetection();

        /**
         * Check for multiple valid detections
         */
        boolean hasMultipleDetections();

        /**
         * Get number of current detections
         */
        int getDetectionCount();

        /**
         * Check if current state has timed out
         */
        boolean isTimeout();

        /**
         * Check if error recovery is possible
         */
        boolean canRetry();

        /**
         * Get time in current state
         */
        long getStateTime();

        /**
         * Get detection confidence level
         */
        double getConfidence();

        /**
         * Get average confidence across all detections
         */
        default double getAverageConfidence() {
                return hasMultipleDetections() ? getMultipleConfidences().stream()
                                .mapToDouble(Double::doubleValue)
                                .average()
                                .orElse(0.0) : getConfidence();
        }

        /**
         * Get confidence levels for all detections
         */
        java.util.List<Double> getMultipleConfidences();

        /**
         * Check if pipeline is processing correctly
         */
        boolean isPipelineOperational();

        /**
         * Check if camera feed is valid
         */
        boolean isCameraOperational();

        /**
         * Get current pipeline processing time
         */
        long getProcessingTime();

        /**
         * Check if detection is stable over time
         */
        default boolean hasStableDetection() {
                return hasDetection() && getConfidence() >= 0.8 && getProcessingTime() < 50;
        }

        /**
         * Check if high confidence detection exists
         */
        default boolean hasConfidentDetection() {
                return hasDetection() && getConfidence() >= 0.8;
        }

        /**
         * Check if detection quality is degrading
         */
        default boolean isDetectionDegrading() {
                return hasDetection() && getConfidence() < 0.5;
        }

        /**
         * Check for error conditions
         */
        default boolean hasError() {
                return !isInitialized() ||
                                !isPipelineOperational() ||
                                !isCameraOperational() ||
                                getProcessingTime() > 100;
        }

        /**
         * Get specific error type if error exists
         */
        default VisionState getErrorState() {
                if (!isInitialized())
                        return VisionState.ERROR_PIPELINE;
                if (!isCameraOperational())
                        return VisionState.ERROR_CAMERA;
                if (isTimeout())
                        return VisionState.ERROR_TIMEOUT;
                return null;
        }
}
