package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.utils.constants.TimingConstants;

/**
 * Enhanced state machine for computer vision pipeline management.
 * Handles multiple detection scenarios, confidence-based tracking,
 * and robust error recovery. Supports different pipeline types and
 * provides detailed state information for debugging.
 */
public enum VisionState {
        // Initialization states
        INITIALIZING(TimingConstants.VISION_DETECTION_TIMEOUT, false),
        CALIBRATING(TimingConstants.VISION_CALIBRATION_TIMEOUT, false),

        // Detection states
        SEARCHING(TimingConstants.VISION_DETECTION_TIMEOUT, true),
        DETECTING(TimingConstants.VISION_DETECTION_TIMEOUT, true),
        CONFIRMING(TimingConstants.VISION_CONFIRMATION_TIMEOUT, true),

        // Tracking states
        TRACKING_SINGLE(TimingConstants.VISION_DETECTION_TIMEOUT, true),
        TRACKING_MULTIPLE(TimingConstants.VISION_DETECTION_TIMEOUT, true),
        TRACKING_WEAK(TimingConstants.VISION_WEAK_TIMEOUT, true),

        // Recovery states
        LOST_TRACK(TimingConstants.ERROR_RECOVERY_DELAY, false),
        REACQUIRING(TimingConstants.VISION_REACQUIRE_TIMEOUT, true),

        // Error states
        ERROR_TIMEOUT(TimingConstants.ERROR_RETRY_TIMEOUT, false),
        ERROR_PIPELINE(TimingConstants.ERROR_RETRY_TIMEOUT, false),
        ERROR_CAMERA(TimingConstants.ERROR_RETRY_TIMEOUT, false);

        private final int timeout;
        private final boolean processingActive;

        VisionState(int timeout, boolean processingActive) {
                this.timeout = timeout;
                this.processingActive = processingActive;
        }

        public int getTimeout() {
                return timeout;
        }

        public boolean isProcessingActive() {
                return processingActive;
        }

        /**
         * Determine the next state based on current context
         */
        public VisionState nextState(VisionStateContext context) {
                try {
                        switch (this) {
                                case INITIALIZING:
                                        if (!context.isInitialized())
                                                return INITIALIZING;
                                        return context.needsCalibration() ? CALIBRATING : SEARCHING;

                                case CALIBRATING:
                                        if (context.isCalibrated())
                                                return SEARCHING;
                                        if (context.isTimeout())
                                                return ERROR_CAMERA;
                                        return CALIBRATING;

                                case SEARCHING:
                                        if (context.hasDetection())
                                                return DETECTING;
                                        if (context.isTimeout())
                                                return ERROR_TIMEOUT;
                                        return SEARCHING;

                                case DETECTING:
                                        if (!context.hasDetection())
                                                return SEARCHING;
                                        if (context.getConfidence() > 0.8)
                                                return TRACKING_SINGLE;
                                        if (context.getConfidence() > 0.5)
                                                return CONFIRMING;
                                        return DETECTING;

                                case CONFIRMING:
                                        if (!context.hasDetection())
                                                return SEARCHING;
                                        if (context.getConfidence() > 0.8)
                                                return TRACKING_SINGLE;
                                        if (context.getStateTime() > TimingConstants.VISION_CONFIRMATION_TIMEOUT)
                                                return DETECTING;
                                        return CONFIRMING;

                                case TRACKING_SINGLE:
                                        if (!context.hasDetection())
                                                return LOST_TRACK;
                                        if (context.hasMultipleDetections())
                                                return TRACKING_MULTIPLE;
                                        if (context.getConfidence() < 0.5)
                                                return TRACKING_WEAK;
                                        return TRACKING_SINGLE;

                                case TRACKING_MULTIPLE:
                                        if (!context.hasDetection())
                                                return LOST_TRACK;
                                        if (!context.hasMultipleDetections())
                                                return TRACKING_SINGLE;
                                        if (context.getConfidence() < 0.5)
                                                return TRACKING_WEAK;
                                        return TRACKING_MULTIPLE;

                                case TRACKING_WEAK:
                                        if (!context.hasDetection())
                                                return LOST_TRACK;
                                        if (context.getConfidence() > 0.8)
                                                return TRACKING_SINGLE;
                                        if (context.isTimeout())
                                                return REACQUIRING;
                                        return TRACKING_WEAK;

                                case LOST_TRACK:
                                        if (context.hasDetection())
                                                return DETECTING;
                                        if (context.isTimeout())
                                                return REACQUIRING;
                                        return LOST_TRACK;

                                case REACQUIRING:
                                        if (context.hasDetection())
                                                return DETECTING;
                                        if (context.isTimeout())
                                                return SEARCHING;
                                        return REACQUIRING;

                                case ERROR_TIMEOUT:
                                case ERROR_PIPELINE:
                                case ERROR_CAMERA:
                                        if (!context.canRetry())
                                                return this;
                                        return INITIALIZING;

                                default:
                                        return ERROR_PIPELINE;
                        }
                } catch (Exception e) {
                        return ERROR_PIPELINE;
                }
        }

        /**
         * Check if this state requires pipeline switching
         */
        public boolean requiresPipelineSwitch() {
                return this == SEARCHING ||
                                this == INITIALIZING ||
                                this == CALIBRATING ||
                                this == REACQUIRING;
        }

        /**
         * Get transition delay based on state type
         */
        public int getTransitionDelay() {
                if (this == CALIBRATING)
                        return TimingConstants.VISION_CALIBRATION_DELAY;
                if (this.requiresPipelineSwitch())
                        return TimingConstants.VISION_PIPELINE_SWITCH_DELAY;
                return 0;
        }

        /**
         * Check if state indicates an error condition
         */
        public boolean isErrorState() {
                return this == ERROR_TIMEOUT ||
                                this == ERROR_PIPELINE ||
                                this == ERROR_CAMERA;
        }

        /**
         * Check if state is actively tracking targets
         */
        public boolean isTrackingState() {
                return this == TRACKING_SINGLE ||
                                this == TRACKING_MULTIPLE ||
                                this == TRACKING_WEAK;
        }

        /**
         * Check if state requires high confidence threshold
         */
        public boolean requiresHighConfidence() {
                return this == TRACKING_SINGLE ||
                                this == TRACKING_MULTIPLE ||
                                this == CONFIRMING;
        }
}
