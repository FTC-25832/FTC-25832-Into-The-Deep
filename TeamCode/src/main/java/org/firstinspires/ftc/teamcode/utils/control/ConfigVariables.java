package org.firstinspires.ftc.teamcode.utils.control;

import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.ftcontrol.panels.configurables.Configurables;

@Config
// @Configurable
public class ConfigVariables {
        // this just default/initial values prob newer adjusted ones in ftc dashboard
        // ftc dashboard at :8030/dash

        @Config
        // @Configurable
        public static class General {

                public static long DASHBOARD_UPDATE_INTERVAL_MS = 1;
                public static double DRIVE_ROTATE_FACTOR = 0.5;
                public static double HANGING_SERVOS_SPEED = 0.8; // pwm unit
                public static double DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT = 1;
                public static int DISTANCE_THRESHOLD_ENCODER = 50;
                public static int CLAW_OPERATION_TIMEOUT = 200;
                public static int ARM_OPERATION_TIMEOUT = 400;
        }

        @Config
        public static class HangingTesting {
                public static double pos1 = 0;
                public static double pos2 = -1;
                public static double pos3 = 1;
                public static double pos4 = 0.2;
        }

        @Config
        public static class AutoTesting {
                public static double A_DROPDELAY_S = 0.25;

                public static double B_AFTERSCOREDELAY_S = 0.05;
                public static double C_AFTERGRABDELAY_S = 0.1;
                public static double D_SLIDEPOS0AFTERDELAY_S = 0.1;
                public static double E_LOWSLIDEUPAFTERDELAY_S = 0.35;
                public static double F_TRANSFERAFTERDELAY_S = 0.2;
                public static double G_LOWSLIDETRANSFEROPENCLAWAFTERDELAY_S = 0.05;
                public static double H_TRANSFERCOMPLETEAFTERDELAY_S = 0.2;

                public static double I_SUBDELAY_S = 0.4;
                public static double J_AFTERSUBDELAY_S = 0.4;

                public static double Z_LowerslideExtend_FIRST = 39; //was 20
                public static double Z_LowerslideExtend_SECOND = 39;
                public static double Z_LowerslideExtend_THIRD = 17;

                public static double Y_PICKUPDELAY = Camera.CAMERA_DELAY;

                public static double X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S = 0.1;
                public static double W_AFTEREXTENDOOPEN_S = 0.2;

                // State machine timeouts
                public static final double STATE_TIMEOUT = 5.0;
                public static final double VISION_ALIGN_TIMEOUT = 3.0;
                public static final double GRAB_SEQUENCE_TIMEOUT = 2.0;
                public static final double SCORE_SEQUENCE_TIMEOUT = 2.0;

                // Movement delays
                public static final double AFTER_MOVE_DELAY = 0.3;
                public static final double AFTER_ALIGN_DELAY = 0.2;
                public static final double AFTER_GRAB_DELAY = 0.5;
                public static final double AFTER_SCORE_DELAY = 0.5;

        }

        @Config
        public static class Camera {
                public static double CAMERA_HEIGHT = 27; // cm, height of camera from ground
                public static double CLAW_DISTANCE = 22; // cm
                public static double CROSSHAIR_X = -0.42578125;
                public static final double ADJUSTMENT_DELAY = 0.65; // 500ms between adjustments
                public static double CROSSHAIR_Y = -0.4791666567325592;
                public static double TILT_ANGLE = 45.0;
                public static double[][] CAMERA_MATRIX = {
                        {1221.445, 0, 637.226},
                        {0, 1223.398, 502.549},
                        {0, 0, 1}
                };
                public static double[] DISTORTION_COEFFS = {0.177168, -0.457341, 0.000360, 0.002753, 0.178259};
                public static double TOLERANCE = 2.5;
                public static double Y_OFFSET = 0; //cm
                public static double CAMERA_DELAY = 0.2; // s
                public static double CLAW_90 = 90;
                public static double XYPIXELRATIO = 225.0 / 672.0;
                public static double XYDISTANCERATIO = 2.2/6.7;
                public static double CAMERA_DISTANCE = 27; // cm, y distance between camera and sample
                public static double[] Y_DISTANCE_MAP_X = {
                                -100,
                                -7, -5.4, -4.3, -3.5, -2,
                                1, 2.1, 5.5, 8.7, 10,
                                12.4, 14.2, 16.5, 19.7, 21,
                                22.3, 23.0, 23.5, 24.1, 27.1, 100
                };
                public static double[] Y_DISTANCE_MAP_Y = {
                                12.5,
                                12.5, 14.5, 15.5, 16, 18.5,
                                21.5, 23.5, 25.5, 27.5, 28.5,
                                30.5, 33, 35, 38.5, 41,
                                42.3, 44, 45, 45.5, 50, 50
                };
                public static double[] X_DISTANCE_MAP_X = {
                                -100,
                                -11.6, -11, -9.4, -4.5, -2.3,
                                0,
                                3.2, 5, 7, 11.8, 15.4, 20.1,
                                25.4, 30.55, 35, 36,
                                100
                };
                public static double[] X_DISTANCE_MAP_Y = {
                                -13,
                                -10.25, -9, -6, -2.5, -1.25,
                                0,
                                1.25, 2.75, 3.75, 5.6, 8, 10.5,
                                12.8, 15.8, 18.8, 20,
                                24.3
                };
                public static String[] ACCEPTED_COLORS = {
                                "blue", "red", "yellow"
                };
                public static int ANGLE_MAXNUM = 5;
                // public static int PID_UPDATE_TIMEOUT = 500;
                public static int ANGLE_OFFSET = 100;
                public static double PID_KP = 0.008;
                public static double PID_KI = 0.002;
                public static double PID_KD = 0.0;
                public static double PID_KF = 0.0;
                public static double DISTANCE_THRESHOLD = 1;

        }

        // UpperSlide
        @Config
        // @Configurable
        public static class UpperSlideVars {
                // Arm positions
                public static double FRONT_ARM_POS = 0.7;
                public static double FRONT_SWING_POS = 0.3;
                public static double BEHIND_ARM_POS = 0.24;
                public static double BEHIND_SWING_POS = 1.0;

                public static double INTER_ARM_POS = 0.5;
                public static double INTER_SWING_POS = 0.8;

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
                public static double OFFWALL_FRONT_ARM_POS = 1;
                public static double OFFWALL_FRONT_SWING_POS = 0.65;

                // scorespec positions
                public static double SCORESPEC_FRONT_ARM_POS = 0.35;
                public static double SCORESPEC_FRONT_SWING_POS = 0.65;

                public static double PID_KP = 0.01;
                public static double PID_KI = 0.0;
                public static double PID_KD = 0.0;
                public static double PID_KF = 0.0; // Feedforward gain for gravity compensation
        }

        // LowerSlide
        @Config
        // @Configurable
        public static class LowerSlideVars {
                // Arm positions
                public static double GRAB_BIG = 0.93;
                public static double GRAB_SMALL = 0;
                public static double UP_BIG = 0.52;
                public static double UP_SMALL = 1.0;
                public static double HOVER_BIG = 0.7;
                public static double HOVER_SMALL = 0.1;

                // slide positions
                public static double POS_0_CM = 0;
                public static double POS_1_CM = 35; //max is 45
                public static double POS_2_CM = 10;

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
                // public static double PID_KF = 0.0;
        }
}
