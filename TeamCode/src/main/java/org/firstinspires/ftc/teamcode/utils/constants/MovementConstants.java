package org.firstinspires.ftc.teamcode.utils.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants related to robot movement, dimensions, and trajectories.
 * Using @Config annotation to allow live tuning through FTC Dashboard.
 */
@Config
public class MovementConstants {
        // Robot physical dimensions (in inches)
        public static class RobotDimensions {
                public static double BOT_LENGTH = 15.748;
                public static double BOT_WIDTH = 13.386;
                public static double TRACK_WIDTH = 11.25286365;

                // Dead wheel positions
                public static double PAR0_Y_TICKS = -9690.675784657145; // y position of first parallel encoder
                public static double PAR1_Y_TICKS = 9773.439152317475; // y position of second parallel encoder
                public static double PERP_X_TICKS = -2572.8974384671237; // x position of perpendicular encoder
        }

        // Movement constraints
        public static class DriveConstraints {
                public static double MAX_VEL = 90; // Maximum velocity in inches per second
                public static double MAX_ACCEL = 70; // Maximum acceleration in inches per second squared
                public static double MAX_ANG_VEL = 55; // Maximum angular velocity in degrees per second
                public static double MAX_ANG_ACCEL = 60;// Maximum angular acceleration in degrees per second squared

                // Slide movement constraints
                public static double MAX_SLIDE_VEL = 30; // Maximum slide velocity
                public static double MAX_SLIDE_ACCEL = 20; // Maximum slide acceleration
                public static double SLIDE_EXTENSION_LIMIT = 45.0; // Maximum slide extension in cm
        }

        // Field positions (in inches)
        public static class FieldPositions {
                // Y-axis field positions
                public static double TILE_SIZE = 24.0; // Size of one field tile
                public static double TEST_Y_VALUE = 61.5;
                public static double TEST_Y_VALUE2 = 33.0;
                public static double TEST_Y_VALUE3 = 61.5;
                public static double TEST_Y_VALUE4 = 33.0;

                // Offsets and adjustments
                public static double THIRD_SPECIMEN_OFFSET = 3.5;
                public static double FOURTH_SPECIMEN_OFFSET = 3.5;
                public static double CLIP_OFFSET = 3.5;
                public static double TEST_X_VALUE = -45.0;
        }

        // PID and Control Constants
        public static class ControlConstants {
                // Drive PID
                public static double DRIVE_P = 0.05;
                public static double DRIVE_I = 0.0;
                public static double DRIVE_D = 0.0;
                public static double DRIVE_F = 0.0;

                // Turn PID
                public static double TURN_P = 0.05;
                public static double TURN_I = 0.0;
                public static double TURN_D = 0.0;
                public static double TURN_F = 0.0;

                // Slide PID
                public static double SLIDE_P = 0.05;
                public static double SLIDE_I = 0.0;
                public static double SLIDE_D = 0.0;
                public static double SLIDE_F = 0.0;
        }

        // Robot orientation constants
        public static class Orientation {
                public static double NEUTRAL_PITCH = 0.15;
                public static double NEUTRAL_YAW = 1.0;

                // Common angles in degrees
                public static double FORWARD = 0.0;
                public static double RIGHT = 90.0;
                public static double BACK = 180.0;
                public static double LEFT = 270.0;
        }

        private MovementConstants() {
                // Private constructor to prevent instantiation
        }
}
