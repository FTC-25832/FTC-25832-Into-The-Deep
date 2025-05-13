package org.firstinspires.ftc.teamcode.utils.control;

public class ControlHub {
        private static final String[] motorport = {
                        "rightBack", // slide 1 motor, leftpod encoder
                        "slide2",
                        "leftFront",
                        "leftBack"
        };

        private static final String[] servoport = {
                        "swervo",
                        "PodM",
                        "servofront",
                        "long1",
                        "long2",
                        "servo"
        };

        public static String motor(int port) {
                return motorport[port];
        }

        public static String servo(int port) {
                return servoport[port];
        }

}
