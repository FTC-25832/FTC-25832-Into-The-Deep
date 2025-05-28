package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

@TeleOp(group = "Test")
public class TestLowslideEncoder extends LinearOpMode {
    private LowerSlide lowslide;
    private double lastResetPosition = 0;
    private double lastResetPositionCM = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        lowslide.initialize(hardwareMap);
        while (opModeIsActive()){
            telemetry.addData("Abs position Encoder", lastResetPosition + lowslide.getCurrentPosition());
            telemetry.addData("Abs position CM", lastResetPositionCM + lowslide.getCurrentPositionCM());
            telemetry.addData("Last reset position Encoder", lastResetPosition);
            telemetry.addData("Last reset  CM", lastResetPositionCM);
            telemetry.addData("position Encoder", lowslide.getCurrentPosition());
            telemetry.addData("position CM", lowslide.getCurrentPositionCM());
            if(gamepad1.x){
                lastResetPosition = lowslide.getCurrentPosition();
                lastResetPositionCM = lowslide.getCurrentPositionCM();
                lowslide.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lowslide.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            telemetry.update();
        }
    }

}
