package org.firstinspires.ftc.teamcode.utils.control;

import com.acmerobotics.roadrunner.Pose2d;

public class PositionConstants {
        // Starting positions
        public static final Pose2d START_POS = new Pose2d(38.93, 60.23, Math.toRadians(180));

        // Sample positions
        public static final Pose2d PREPLACE_POS = new Pose2d(38.93, 60.23, Math.toRadians(180));
        public static final Pose2d[] SAMPLE_POSITIONS = {
                        new Pose2d(38.93, 60.23, Math.toRadians(180)),
                        new Pose2d(38.93, 60.23, Math.toRadians(180)),
                        new Pose2d(38.93, 60.23, Math.toRadians(180))
        };

        // Scoring positions
        public static final Pose2d SCORE_POS = new Pose2d(38.93, 60.23, Math.toRadians(180));

        // Parking positions
        public static final Pose2d PARK_POS = new Pose2d(38.93, 60.23, Math.toRadians(180));

        // Vision alignment positions
        public static final double VISION_ALIGN_X_THRESHOLD = 0.5;
        public static final double VISION_ALIGN_Y_THRESHOLD = 0.5;
        public static final double VISION_ALIGN_ANGLE_THRESHOLD = Math.toRadians(2.0);
}