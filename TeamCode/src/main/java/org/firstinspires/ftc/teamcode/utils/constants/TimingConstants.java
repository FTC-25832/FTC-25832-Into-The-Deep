package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants related to timing operations like delays, timeouts, and durations.
 * Using @Config annotation to allow live tuning through FTC Dashboard.
 */
@Config
public class TimingConstants {
        // Autonomous timing constants (in milliseconds)
        public static int CLIP_DELAY = 200;
        public static int GRAB_DELAY = 100;
        public static int PICKUP_DELAY = 200;
        public static int DROP_OFF_DELAY = 200;
        public static double EXTEND_DELAY = 1;
        public static int EXTEND_LENGTH = 515;

        // Vision system timeouts and delays
        public static int VISION_DETECTION_TIMEOUT = 1000;
        public static int VISION_PIPELINE_SWITCH_DELAY = 50;
        public static int VISION_CALIBRATION_TIMEOUT = 3000;
        public static int VISION_CALIBRATION_DELAY = 100;
        public static int VISION_CONFIRMATION_TIMEOUT = 500;
        public static int VISION_REACQUIRE_TIMEOUT = 750;
        public static int VISION_WEAK_TIMEOUT = 250;

        // Vision confidence thresholds
        public static double VISION_HIGH_CONFIDENCE = 0.8;
        public static double VISION_MEDIUM_CONFIDENCE = 0.5;
        public static double VISION_LOW_CONFIDENCE = 0.3;

        // Vision performance monitoring
        public static int VISION_FRAME_TIMEOUT = 100;
        public static int VISION_MIN_FPS = 15;
        public static int VISION_TARGET_FPS = 30;
        public static int VISION_PROCESS_WARNING = 33; // ms (for 30 FPS)
        public static int VISION_PROCESS_CRITICAL = 66; // ms (for 15 FPS)

        // Movement timing constraints (in milliseconds)
        public static int TRAJECTORY_TIMEOUT = 5000;
        public static int TURN_TIMEOUT = 2000;
        public static int SLIDE_MOVEMENT_TIMEOUT = 1000;

        // Pre-match system checks
        public static int SYSTEM_CHECK_TIMEOUT = 500;
        public static int IMU_INIT_TIMEOUT = 2000;

        // Recovery timing
        public static int ERROR_RECOVERY_DELAY = 100;
        public static int ERROR_RETRY_TIMEOUT = 3000;

        // Performance monitoring thresholds
        public static int LOOP_TIME_WARNING_THRESHOLD = 50; // ms
        public static int LOOP_TIME_CRITICAL_THRESHOLD = 100; // ms
        public static int TARGET_LOOP_TIME = 7; // Target time per loop (ms) for 150+ Hz
        public static int BULK_READ_WARNING_THRESHOLD = 5; // ms
        public static int CACHE_CLEAR_WARNING_THRESHOLD = 2; // ms
        public static int TELEMETRY_UPDATE_INTERVAL = 50; // ms
        public static int PERFORMANCE_SAMPLE_SIZE = 50; // Rolling average window size
        public static int VISION_SAMPLE_SIZE = 10; // Vision metrics sample size

        // Dead wheel odometry
        public static int ODOMETRY_UPDATE_THRESHOLD = 10; // ms

        // State transitions
        public static class StateTransition {
                public static int DEFAULT_SETTLING_TIME = 250;
                public static int POSITION_SETTLING_TIME = 500;
                public static int HEADING_SETTLING_TIME = 750;

                public static double POSITION_TOLERANCE = 0.5; // inches
                public static double HEADING_TOLERANCE = 2.0; // degrees
        }

        // Motor control
        public static class MotorControl {
                public static int MIN_MOVE_DURATION = 50; // Minimum time for a move to avoid jerky motion
                public static int PID_UPDATE_RATE = 50; // How often to update PID calculations
                public static int STALL_DETECTION_TIME = 500; // Time to detect a stalled motor
                public static double STALL_THRESHOLD = 0.1; // Movement threshold for stall detection
        }

        private TimingConstants() {
                // Private constructor to prevent instantiation
        }
}
