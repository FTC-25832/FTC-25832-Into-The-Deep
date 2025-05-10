package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

@TeleOp(group = "Test")
public class TestUpperSlide extends LinearOpMode {

    UpperSlide slide = new UpperSlide();
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean cPressed = false;
    private boolean dPressed = false;

    @Override
    public void runOpMode() {
        slide.initialize(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                slide.pos0();
            }
            if (gamepad1.x) {
                slide.pos1();
            }
            if (gamepad1.y) {
                slide.pos2();
            }
            if (gamepad1.b) {
                slide.pos3();
            }
            if (gamepad1.right_bumper) {
                if (!aPressed) {
                    slide.addArmPos(0.05);
                    aPressed = true;
                }
            } else {
                aPressed = false;
            }

            if (gamepad1.left_bumper) {
                if (!bPressed) {
                    slide.addArmPos(-0.05);
                    bPressed = true;
                }
            } else {
                bPressed = false;
            }

            if (gamepad1.right_trigger > 0) {
                if (!cPressed) {
                    slide.addSwingPos(0.05);
                    cPressed = true;
                }
            } else {
                cPressed = false;
            }

            if (gamepad1.left_trigger > 0) {
                if (!dPressed) {
                    slide.addSwingPos(-0.05);
                    dPressed = true;
                }
            } else {
                dPressed = false;
            }

            double power = slide.updatePID();

            if (gamepad1.dpad_left) {
                slide.closeClaw();
            }
            if (gamepad1.dpad_right) {
                slide.openClaw();
            }

            double currentPos = (slide.slide1Encoder.getCurrentPosition() + slide.slide2Encoder.getCurrentPosition())
                    / 2.0;
            double targetPos = slide.pidController.destination;
            double error = targetPos - currentPos;
            double feedforward = slide.pidController.kf * targetPos;

            telemetry.addData("Current Position", currentPos);
            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Error", error);
            telemetry.addData("Feedforward", feedforward);
            telemetry.addData("Total Power", power);
            telemetry.addData("PID Constants", "kP=%.3f, kI=%.3f, kD=%.3f, kF=%.3f",
                    slide.pidController.kp,
                    slide.pidController.ki,
                    slide.pidController.kd,
                    slide.pidController.kf);
            telemetry.addData("arm1", slide.arm1.getPosition());
            telemetry.addData("arm2", slide.arm2.getPosition());
            telemetry.addData("claw", slide.claw.getPosition());
            telemetry.addData("swing", slide.swing.getPosition());
            telemetry.update();
        }
    }
}
