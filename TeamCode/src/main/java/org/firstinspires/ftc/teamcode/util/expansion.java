package org.firstinspires.ftc.teamcode.util;

public class expansion {
        private static String motorport[] = {
                        "rightBack", // slide1 encoder
                        "rightFront", // lowerslide encoder
                        "leftFront",
                        "leftBack"
        };

        private static String servoport[] = {
                        "spinclaw",
                        "a",
                        "b",
                        "front",
                        "down2",
                        "swing"
        };

        public static String motor(int port) {
                return motorport[port];
        }

        public static String servo(int port) {
                return servoport[port];
        }

}
