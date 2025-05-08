package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import static org.firstinspires.ftc.teamcode.util.ConfigVariables.LowerSlideVars;

public class Hanging {
    HardwareMap hardwareMap;
    public ServoImplEx left, right;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);


    public void initialize(HardwareMap map) {
        hardwareMap = map;

        // Initialize servos
        left = hardwareMap.get(ServoImplEx.class, expansion.servo(5));
        right = hardwareMap.get(ServoImplEx.class, control.servo(5));
        right.setDirection(Servo.Direction.REVERSE);
        right.setPwmRange(v4range);
        left.setDirection(Servo.Direction.FORWARD);
        left.setPwmRange(v4range);
    }
  public void turnForward(){
    left.setPosition(ConfigVariables.General.HANGING_SERVOS_SPEED);
    right.setPosition(ConfigVariables.General.HANGING_SERVOS_SPEED);
  }
  public void stop(){
      left.setPosition(0);
      right.setPosition(0);
  }
  public void turnBackward(){
      left.setPosition(-ConfigVariables.General.HANGING_SERVOS_SPEED);
      right.setPosition(-ConfigVariables.General.HANGING_SERVOS_SPEED);
  }

}
