package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

@Config
public static class PIDConstant {
    public static double Kp = 0.01; // Proportional gain
    public static double Ki = 0.0; // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0.0; // Derivative gain
}