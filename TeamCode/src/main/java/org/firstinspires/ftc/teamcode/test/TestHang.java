package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

@TeleOp(group = "Test")
public class TestHang extends LinearOpMode {
    private Hanging hangingServos;
    @Override
    public void runOpMode() throws InterruptedException {
        hangingServos = new Hanging();
        hangingServos.initialize(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                hangingServos.setPwm(ConfigVariables.HangingTesting.pos1);
            }
            if(gamepad1.b){
                hangingServos.setPwm(ConfigVariables.HangingTesting.pos2);
            }
            if(gamepad1.x){
                hangingServos.setPwm(ConfigVariables.HangingTesting.pos3);
            }
            if(gamepad1.y){
                hangingServos.setPwm(ConfigVariables.HangingTesting.pos4);
            }
        }
    }

}
