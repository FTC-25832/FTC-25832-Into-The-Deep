package org.firstinspires.ftc.teamcode.util;

public class control {
        private static String motorport[] = {
                        "slide1", // slide 1 motor, leftpod encoder
                        "slide2",
                        "podM",
                        "podL"
        };

        private static String servoport[] = {
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
