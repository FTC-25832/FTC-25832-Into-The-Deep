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

        public static int CLAW_FORTESTING_DEG = 55; // deg
        public static double HANGING_SERVOS_SPEED = 10; // pwm unit

        public static double DRIVETRAIN_SPEED_MULTIPLIERFORLIMIT = 0.5;
    }

    @Config
    public static class AutoTesting {
        public static int lowerslideextendlength = 27;
        public static double DROPDELAY_S = 0.5;
    }

    @Config
    public static class Camera {
        // public static double CLAW_DISTANCE = 20; //cm
        // public static double[] DISTANCE_MAP = { CLAW_DISTANCE, CLAW_DISTANCE,
        // CLAW_DISTANCE, CLAW_DISTANCE, CLAW_DISTANCE, CLAW_DISTANCE, CLAW_DISTANCE,
        // 20, 21, 22.5, 23.5, 24.3, 25.5, 26.5, 27.5, 28.5, 30, 31.5, 32.7, 34, 35.5,
        // 37, 38, 40, 41.5, 43, 44, 45.5, 0, 0, 0 };
        public static int ANGLE_MAXNUM = 20;
        // public static int PID_UPDATE_TIMEOUT = 500;
        public static int ANGLE_OFFSET = 55;
        public static double PID_KP = 0.01;
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
        public static double FRONT_ARM_POS = 0.4;
        public static double FRONT_SWING_POS = 0.4;
        public static double BEHIND_ARM_POS = 0.20;
        public static double BEHIND_SWING_POS = 1.0;

        // Claw positions
        public static double CLAW_OPEN = 1.0;
        public static double CLAW_CLOSE = 0.0;

        // Slide positions (in cm)
        public static double POS_0_CM = 0.0;
        public static double POS_PRE_0_CM = 10.0;
        public static double POS_1_CM = 25.0;
        public static double POS_2_CM = 42.0;
        public static double POS_3_CM = 40.0;

        // offwall positions
        public static double OFFWALL_FRONT_ARM_POS = 0.85;
        public static double OFFWALL_FRONT_SWING_POS = 0.80;

        // scorespec positions
        public static double SCORESPEC_FRONT_ARM_POS = 0.23;
        public static double SCORESPEC_FRONT_SWING_POS = 0.65;

        public static double PID_KP = 0.01;
        public static double PID_KI = 0.01;
        public static double PID_KD = 0.0;
        public static double PID_KF = 0.0; // Feedforward gain for gravity compensation
    }

    // LowerSlide
    @Config
    // @Configurable
    public static class LowerSlideVars {
        // Arm positions
        public static double GRAB_BIG = 0.93;
        public static double GRAB_SMALL = 0.1;
        public static double UP_BIG = 0.55;
        public static double UP_SMALL = 1.0;
        public static double HOVER_BIG = 0.7;
        public static double HOVER_SMALL = 0.1;

        // slide positions
        public static double POS_0_CM = 0;
        public static double POS_1_CM = 50;
        public static double POS_2_CM = 0;

        // Claw positions
        public static double CLAW_OPEN = 1.0;
        public static double CLAW_CLOSE = 0.0;

        public static int POS_GRAB_TIMEOUT = 200;
        public static int CLAW_CLOSE_TIMEOUT = 200;
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
