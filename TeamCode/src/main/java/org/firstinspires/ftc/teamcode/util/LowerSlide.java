package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class LowerSlide {

    static final double     PI=3.14;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
//    static final double     WHEEL_CIRCUMFERENCE_MM  = 40.0 * PI;
//    static final double     DRIVE_GEAR_REDUCTION    = 18.88;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 37.0 * PI;
    static final double     DRIVE_GEAR_REDUCTION    = 4.0;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_CM           = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM)*10;

    HardwareMap hardwareMap;
    public ServoImplEx part1, part2, part3, spinclaw, claw;

    public DcMotor slide;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange downrange = new PwmControl.PwmRange(500, 900);

    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1150);

    private PIDController pidController;
    public void initialize(HardwareMap map) {
        hardwareMap = map;
        
        // 初始化PID控制器
        pidController = new PIDController(0.02, PIDConstant.Ki, PIDConstant.Kd);

        part1 = hardwareMap.get(ServoImplEx.class, expansion.servo(3));
        part2 = hardwareMap.get(ServoImplEx.class, control.servo(0));
        //part3 = hardwareMap.get(ServoImplEx.class, "cyliis");

        //slide = hardwareMap.get(DcMotor.class, "down1");
        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = hardwareMap.get(DcMotor.class, control.motor(2));
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spinclaw = hardwareMap.get(ServoImplEx.class, expansion.servo(2));
        claw = hardwareMap.get(ServoImplEx.class, expansion.servo(0));

        //part2.setDirection(ServoImplEx.Direction.REVERSE);
        part1.setDirection(Servo.Direction.FORWARD);
        part2.setDirection(Servo.Direction.FORWARD);
        part1.setPwmRange(v4range);
        part2.setPwmRange(v4range);
        //part3.setPwmRange(v4range);
        spinclaw.setPwmRange(v4range);
        claw.setPwmRange(clawRange);



    }

    public void low(double val){
        slide.setTargetPosition((int)val);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setSlidePower(double power){
        slide.setPower(power);
    }

    public void keepPosExceptArms(double pos) {
        part1.setPosition(0);
        part2.setPosition(0);
    }

    public void big(double val){ part1.setPosition(val); }
    public void small(double val){ part2.setPosition(val); }

    public void smallclaw(double val){ part3.setPosition(val); }

    // abandoned
//    public void pos0(){ spinclaw.setPosition(1);}
//    public void pos1(){ spinclaw.setPosition(0.25);}
//    public void pos2(){ spinclaw.setPosition(0.5);}
//    public void pos3(){ spinclaw.setPosition(0.75);}

    public void spinclawSetPositionDeg( double degree ){
        spinclaw.setPosition(degree/270);
    }

    public void pos4(){ spinclaw.setPosition(0);}

    public void pos_grab() {
        big(0.93);
        small(0.1);
    }
    public void pos_up() {
        big(0.6);
        small(1);
    }

    public void pos_hover() {
        big(0.7);
        small(0.1);
    }

    public void setSlidePos1(){
        pidController.setDestination(Math.round(COUNTS_PER_CM * 50));
    }

    public void setSlidePos2(){
        pidController.setDestination(0);
    }

    public void updatePID() {
        double power = pidController.calculate(slide.getCurrentPosition());
        slide.setPower(power);
    }

    public void closeClaw(){
        claw.setPosition(1);
    }
    public void openClaw(){
        claw.setPosition(0);
    }

}
