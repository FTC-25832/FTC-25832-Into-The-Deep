package org.firstinspires.ftc.teamcode.test;

import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.expansion;
import org.firstinspires.ftc.teamcode.util.control;
import org.opencv.video.SparsePyrLKOpticalFlow;

@TeleOp(group="Test")
public class TestLower extends LinearOpMode {

    LowerSlide lowslide = new LowerSlide();
    @Override
    public void runOpMode() {

        lowslide.initialize(hardwareMap);
        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a) {
                lowslide.spinclawSetPositionDeg(0);
            }
            if(gamepad1.b) {
                lowslide.spinclawSetPositionDeg(90);
            }
            if(gamepad1.x) {
                lowslide.spinclawSetPositionDeg(180);
            }
            if(gamepad1.y) {
                lowslide.spinclawSetPositionDeg(270);
            }
            if (gamepad1.left_bumper){
                lowslide.setPositionCM(10);
            }
            if (gamepad1.left_trigger > 0){
                lowslide.setPositionCM(0);
            }
            if(gamepad1.right_bumper){
                lowslide.setPositionCM(20);
            }
            if(gamepad1.right_trigger > 0){
                lowslide.setPositionCM(30);
            }
            lowslide.updatePID();
            //telemetry.addData("part3",part3.getPosition());

            //telemetry.addData("slide1",slide1.getPosition());
            //telemetry.addData("slide2",slide2.getPosition());


            telemetry.update();
        }
    }

}
