/*
Copyright 2024 FIRST Tech Challenge Team 25832

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp(group="Test")
public class TestServo extends LinearOpMode {
    ServoImplEx swervo;
    Servo servo;
    PwmControl.PwmRange range = new PwmControl.PwmRange(500, 2500);
    @Override
    public void runOpMode() {
        swervo = hardwareMap.get(ServoImplEx.class, "swervo");
        swervo.setPwmRange(range);

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
        
            telemetry.addData("Status", "Running");
            
            telemetry.addData("swervo", gamepad1.left_stick_x);
            telemetry.addData("servo", gamepad1.right_stick_x);
            swervo.setPosition(gamepad1.left_stick_x);
            servo.setPosition(gamepad1.right_stick_x);
            
            /*
            if(gamepad1.y){
                swervo.setPosition(0.5);
            }
            if(gamepad1.x){
                swervo.setPosition(1);
            }
            */
            telemetry.update();
        }
    }
}
