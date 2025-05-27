package org.firstinspires.ftc.teamcode.utils.constants;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import java.util.HashMap;
import java.util.Map;

/**
 * Constants related to vision processing and AprilTag configurations.
 */
public class VisionConstants {
        // Camera parameters
        public static class Camera {
                public static final int WIDTH = 640;
                public static final int HEIGHT = 480;
                public static final int FPS = 30;
                public static final String DEFAULT_NAME = "Webcam 1";

                // Camera mount position relative to robot center (in inches)
                public static final double MOUNT_X = 4.0; // Forward/back
                public static final double MOUNT_Y = 0.0; // Left/right
                public static final double MOUNT_Z = 7.5; // Up/down
                public static final double MOUNT_PITCH = 0.0; // Degrees
                public static final double MOUNT_ROLL = 0.0; // Degrees
                public static final double MOUNT_YAW = 0.0; // Degrees
        }

        // AprilTag parameters
        public static class AprilTag {
                public static final double TAG_SIZE = 0.166; // Tag size in meters
                public static final int TAG_FAMILY = 36; // Tag family (36h11)
                public static final int MAX_HAMMING = 2; // Maximum allowable hamming distance

                // Detection thresholds
                public static final float DECIMATION_HIGH = 3;
                public static final float DECIMATION_LOW = 2;
                public static final float THRESHOLD_HIGH = 100;
                public static final float THRESHOLD_LOW = 80;
                public static final double MIN_CONFIDENCE = 0.7;
        }

        private VisionConstants() {
                // Private constructor to prevent instantiation
        }
}
