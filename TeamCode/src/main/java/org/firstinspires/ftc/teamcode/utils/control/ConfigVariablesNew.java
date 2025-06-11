package org.firstinspires.ftc.teamcode.utils.control;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVariablesNew {
        @Config
        public static class General {
                public static long DASHBOARD_UPDATE_INTERVAL_MS = 1;
                public static double DRIVE_ROTATE_FACTOR = 0.5;
                public static double HANGING_SERVOS_SPEED = 0.8;
                public static double DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT = 1;
                public static int DISTANCE_THRESHOLD_ENCODER = 50;
                public static int CLAW_OPERATION_TIMEOUT = 300;
                public static int ARM_OPERATION_TIMEOUT = 400;
        }

        @Config
        public static class Camera {
                public static double CLAW_DISTANCE = 22; // cm
                public static double CLAW_90 = 90;
                public static double CAMERA_DISTANCE = 27; // cm
                public static double[] Y_DISTANCE_MAP_X = {
                                -100, -7, -5.4, -4.3, -3.5, -2, 1, 2.1, 5.5, 8.7, 10,
                                12.4, 14.2, 16.5, 19.7, 21, 22.3, 23.0, 23.5, 24.1, 27.1, 100
                };
                public static double[] Y_DISTANCE_MAP_Y = {
                                12.5, 12.5, 14.5, 15.5, 16, 18.5, 21.5, 23.5, 25.5, 27.5, 28.5,
                                30.5, 33, 35, 38.5, 41, 42.3, 44, 45, 45.5, 50, 50
                };
                public static double[] X_DISTANCE_MAP_X = {
                                -100, -13, -12.5, -10.4, -8.6, -5.6, -3.5, 0, 2, 4.2, 6.4, 9.4, 12.4,
                                15.4, 18.4, 22.5, 25, 28.5, 31.4, 34.0, 35.5, 37, 100
                };
                public static double[] X_DISTANCE_MAP_Y = {
                                -13, -13, -11.5, -9.3, -6.5, -3.5, -2, 0, 2, 3.5, 4.8, 5.5, 7.5,
                                9, 10.2, 12.5, 13.5, 15.5, 17.3, 21, 21.8, 24.3, 24.3
                };
                public static String[] ACCEPTED_COLORS = { "blue", "red", "yellow" };
                public static int ANGLE_MAXNUM = 15;
                public static int ANGLE_OFFSET = 100;
                public static double PID_KP = 0.008;
                public static double PID_KI = 0.002;
                public static double PID_KD = 0.0;
                public static double PID_KF = 0.0;
                public static double DISTANCE_THRESHOLD = 1;
        }

        @Config
        public static class UpperSlideVars {
                // Arm positions
                public static double FRONT_ARM_POS = 0.6;
                public static double FRONT_SWING_POS = 0.3;
                public static double BEHIND_ARM_POS = 0.03;
                public static double BEHIND_SWING_POS = 0.60;

                // Claw positions
                public static double CLAW_OPEN = 1.0;
                public static double CLAW_CLOSE = 0.0;
                public static double EXTENDO_OPEN = 0.0;
                public static double EXTENDO_CLOSE = 1.0;

                // Slide positions (in cm)
                public static double POS_0_CM = 0.0;
                public static double POS_PRE_0_CM = 10.0;
                public static double POS_1_CM = 13;
                public static double POS_2_CM = 40;
                public static double POS_3_CM = 65.0;

                // offwall positions
                public static double OFFWALL_FRONT_ARM_POS = 0.95;
                public static double OFFWALL_FRONT_SWING_POS = 0.65;

                // scorespec positions
                public static double SCORESPEC_FRONT_ARM_POS = 0.18;
                public static double SCORESPEC_FRONT_SWING_POS = 0.65;

                public static double PID_KP = 0.01;
                public static double PID_KI = 0.0;
                public static double PID_KD = 0.0;
                public static double PID_KF = 0.0;
        }

        @Config
        public static class LowerSlideVars {
                // Arm positions
                public static double GRAB_BIG = 0.93;
                public static double GRAB_SMALL = 0;
                public static double UP_BIG = 0.57;
                public static double UP_SMALL = 1.0;
                public static double HOVER_BIG = 0.7;
                public static double HOVER_SMALL = 0.1;

                // slide positions
                public static double POS_0_CM = 0;
                public static double POS_1_CM = 35;
                public static double POS_2_CM = 23;

                // Claw positions
                public static double CLAW_OPEN = 1.0;
                public static double CLAW_CLOSE = 0.0;

                public static int POS_GRAB_TIMEOUT = 200;
                public static int CLAW_CLOSE_TIMEOUT = General.CLAW_OPERATION_TIMEOUT;
                public static int POS_HOVER_TIMEOUT = 200;

                // spin claw positions angle degrees
                public static int ZERO = 0;
                public static int SPINCLAW_DEG = 45;
                public static double PID_KP = 0.01;
                public static double PID_KI = 0.0;
                public static double PID_KD = 0.0;
        }
}