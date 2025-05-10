package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(group="Test")
public class TestMotor extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        DcMotor motor1 = hardwareMap.get(DcMotor.class, "slide2");

        double motor1v = 0.0;
        double eps=0.5;

        //DcMotor testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
//        testMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        //cameraStart();
        waitForStart();
        while(opModeIsActive()) {
            //testMotor.setPower(gamepad1.left_stick_x);

            if (gamepad2.x) {
                motor1v = eps;
            }
            else if (gamepad2.y) {
                motor1v = -eps;
            } else {
                motor1v = 0;
            }
            motor1.setPower(motor1v);

            telemetry.addData("motv", "(%.2f)", motor1v);



//            telemetry.addData("Motor Position",testMotor.getCurrentPosition());
//            telemetry.addData("Gamepad",gamepad1.left_stick_x);
//            telemetry.addData("Status", "Running");
//            if(gamepad1.right_bumper) {
//                frontLeftMotor.setPower(1);
//                sleep(2000);
//                frontLeftMotor.setPower(0);
//
//                frontRightMotor.setPower(1);
//                sleep(2000);
//                frontRightMotor.setPower(0);
//
//                backLeftMotor.setPower(1);
//                sleep(2000);
//                backLeftMotor.setPower(0);
//
//                backRightMotor.setPower(1);
//                sleep(2000);
//                backRightMotor.setPower(0);
//            }
            telemetry.update();
        }
    }
}