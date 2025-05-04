package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigVariables {
        // this just default/initial values prob newer adjusted ones in ftc dashboard
        // ftc dashboard at :8030/dash

        @Config
        public static class General {
                public static int CAMERA_INTERVAL = 1000; // in milliseconds
                public static int CLAW_ZERO_DEG = 55; // deg
        }

        // UpperSlide
        @Config
        public static class UpperSlideVars {
                // Arm positions
                public static double FRONT_ARM_POS = 0.60;
                public static double FRONT_SWING_POS = 0.3;
                public static double BEHIND_ARM_POS = 0.3;
                public static double BEHIND_SWING_POS = 0.0;

                // Claw positions
                public static double CLAW_OPEN = 1.0;
                public static double CLAW_CLOSE = 0.0;

                // Slide positions (in cm)
                public static double POS_0_CM = 10.0;
                public static double POS_1_CM = 50.0;
                public static double POS_2_CM = 70.0;
                public static double POS_3_CM = 60.0;

                // offwall positions
                public static double OFFWALL_FRONT_ARM_POS = 0.80;
                public static double OFFWALL_FRONT_SWING_POS = 0.2;

                // scorespec positions
                public static double SCORESPEC_FRONT_ARM_POS = 0.60;
                public static double SCORESPEC_FRONT_SWING_POS = 0.3;
        }

        // LowerSlide
        @Config
        public static class LowerSlideVars {
                // Arm positions
                public static double GRAB_BIG = 0.93;
                public static double GRAB_SMALL = 0.1;
                public static double UP_BIG = 0.6;
                public static double UP_SMALL = 1.0;
                public static double HOVER_BIG = 0.7;
                public static double HOVER_SMALL = 0.1;

                // Claw positions
                public static double CLAW_OPEN = 0.0;
                public static double CLAW_CLOSE = 1.0;

                // Slide positions (in cm)
                public static double POS_1_CM = 50.0;
                public static double POS_2_CM = 0.0;

                // spin claw positions angle degrees
                public static int ZERO = 0;

                public static int SPINCLAW_DEG = 45;
        }
}
