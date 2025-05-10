package org.firstinspires.ftc.teamcode.utils.hardware;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.control.ControlHub;

@TeleOp(group="Test")
public class ResetServo extends LinearOpMode {

    public double pos = 0.5;
    private ServoImplEx servo;
    private boolean aPressed = false, bPressed = false;
    @Override
    public void runOpMode() {
        PwmControl.PwmRange swingRange = new PwmControl.PwmRange(500, 2500);
        servo = hardwareMap.get(ServoImplEx.class, ControlHub.servo(0));
        servo.setPosition(pos);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a) {
                if(!aPressed) {
                    addPos(0.05);
                    aPressed = true;
                }
            } else {
                aPressed = false;
            }

            if(gamepad2.b) {
                if(!bPressed) {
                    addPos(-0.05);
                    bPressed = true;
                }
            } else {
                bPressed = false;
            }


            telemetry.addData("current position", servo.getPosition());
            telemetry.update();
        }
    }
    public double addPos(double pos){
        double Pos = servo.getPosition();
        Pos += pos;
        Pos = Math.min(1, Math.max(0, Pos));
        servo.setPosition(Pos);
        return Pos;
    }
}

