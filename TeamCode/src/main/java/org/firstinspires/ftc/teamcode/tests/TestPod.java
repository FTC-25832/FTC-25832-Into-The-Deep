package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="Test")
public class TestPod extends LinearOpMode {
    public static double C = (35 * 3.14)/8192;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor pod = hardwareMap.get(DcMotor.class, "slide1");
        pod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double d = (pod.getCurrentPosition())*C;
            telemetry.addData("Distance (mm)",d);
            telemetry.update();
        }
    }
}
