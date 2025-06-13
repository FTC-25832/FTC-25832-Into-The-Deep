package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Shared autonomous path definitions for both robot code and MeepMeep
 * simulation
 */
@Config
public final class AutoPaths {
        // Robot dimensions (in inches)
        public static final double BOT_LENGTH = 15.748;
        public static final double BOT_WIDTH = 13.386;
        public static final double TRACK_WIDTH = 11.25286365;

        // Common robot positions
        public static final class RobotPosition {
                public final Vector2d pos;
                public final double heading;
                public final Pose2d pose;

                public RobotPosition(double x, double y, double headingDegrees) {
                        this.pos = new Vector2d(x, y);
                        this.heading = Math.toRadians(headingDegrees);
                        this.pose = new Pose2d(x, y, this.heading);
                }
        }

        // Standardized robot positions
        public static final RobotPosition START = new RobotPosition(39.5, 65, 180);
        public static final RobotPosition PREPLACED = new RobotPosition(47, 46, -90); //?

        public static final RobotPosition PICKUP1 = new RobotPosition(56, 52, -110);
        public static final RobotPosition PICKUP2 = new RobotPosition(58, 55, -90);
        public static final RobotPosition PICKUP3 = new RobotPosition(47.1, 47.1, -45);
        public static final RobotPosition SCORE = new RobotPosition(57, 57, 225);


        // vvvv these mostly arent used rn vvvv
        // Primary autonomous path points
        public static final double TEST_Y_VALUE = 61.5;
        public static final double TEST_Y_VALUE2 = 33;
        public static final double TEST_Y_VALUE3 = 61.5;
        public static final double TEST_Y_VALUE4 = 33;
        public static final double THIRD_SPECIMEN_OFFSET = 3.5;
        public static final double FOURTH_SPECIMEN_OFFSET = 3.5;
        public static final double CLIP_OFFSET = 3.5;
        public static final double TEST_X_VALUE = -45;

        // Timing constants (in milliseconds)
        public static final int CLIP_DELAY = 200;
        public static final int GRAB_DELAY = 100;
        public static final int PICKUP_DELAY = 200;
        public static final int DROP_OFF_DELAY = 200;
        public static final double EXTEND_DELAY = 1;
        public static final int EXTEND_LENGTH = 515;

        // Robot control parameters
        public static final double NEUTRAL_PITCH = 0.15;
        public static final double NEUTRAL_YAW = 1;

        // Drive constraints, not used in actual auto rn
        public static final double MAX_VEL = 90;
        public static final double MAX_ACCEL = 70;
        public static final double MAX_ANG_VEL = 55;
        public static final double MAX_ANG_ACCEL = 60;
}
