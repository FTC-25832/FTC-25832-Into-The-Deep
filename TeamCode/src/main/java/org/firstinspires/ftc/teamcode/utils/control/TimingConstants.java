package org.firstinspires.ftc.teamcode.utils.control;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TimingConstants {
        // Timing constants (in milliseconds)
        public static final int CLIP_DELAY = 200;
        public static final int GRAB_DELAY = 100;
        public static final int PICKUP_DELAY = 200;
        public static final int DROP_OFF_DELAY = 200;
        public static final double EXTEND_DELAY = 1;
        public static final int EXTEND_LENGTH = 515;

        // Auto testing delays
        public static final double A_DROPDELAY_S = 0.7;
        public static final double B_DROPPEDAFTERDELAY_S = 0.1;
        public static final double C_AFTERGRABDELAY_S = 0.1;
        public static final double D_SLIDEPOS0AFTERDELAY_S = 0.1;
        public static final double E_LOWSLIDEUPAFTERDELAY_S = 0.35;
        public static final double F_TRANSFERAFTERDELAY_S = 0.2;
        public static final double G_LOWSLIDETRANSFEROPENCLAWAFTERDELAY_S = 0.05;
        public static final double H_TRANSFERCOMPLETEAFTERDELAY_S = 0.2;
        public static final double Y_PICKUPDELAY = 1;

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

        // Dashboard update intervals
        public static final double DASHBOARD_UPDATE_INTERVAL_MS = 100.0;
}