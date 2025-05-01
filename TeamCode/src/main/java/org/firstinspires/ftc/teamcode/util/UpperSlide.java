package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Timeout;

import java.sql.Time;


public class UpperSlide {
    HardwareMap hardwareMap;
    public ServoImplEx arm1, arm2, swing, claw;
    PwmControl.PwmRange swingRange = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange armRange = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1270);
    double Kp = PIDConstant.Kp;
    double Ki = PIDConstant.Ki;
    double Kd = PIDConstant.Kd;
    double lastError;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
    static final double     PI=3.14;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 34 * PI;
    static final double     DRIVE_GEAR_REDUCTION    = 5.23;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_CM           = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM)*10;
    public double distance = 0;
    public double ref = 0;
    DcMotor slide1, slide2;
    public void initialize(HardwareMap map) {
        hardwareMap = map;
        slide1 = hardwareMap.get(DcMotor.class, control.motor(0));
        slide2 = hardwareMap.get(DcMotor.class, control.motor(1));

        arm1 = hardwareMap.get(ServoImplEx.class, control.servo(2));
        arm1.setDirection(ServoImplEx.Direction.FORWARD);
        arm1.setPwmRange(armRange);

        arm2 = hardwareMap.get(ServoImplEx.class, expansion.servo(1));
        arm2.setDirection(ServoImplEx.Direction.REVERSE);
        arm2.setPwmRange(armRange);

        swing = hardwareMap.get(ServoImplEx.class, control.servo(1));
        swing.setDirection(ServoImplEx.Direction.FORWARD);
        swing.setPwmRange(swingRange);

        claw = hardwareMap.get(ServoImplEx.class, control.servo(3));
        claw.setPwmRange(clawRange);


        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);

        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void pos0(){
        distance = Math.round(COUNTS_PER_CM*10);
        new Timeout(() -> {
            distance = 0;
        }, 500);
    }
    public void pos1(){
        //closeClaw();
        distance = Math.round(COUNTS_PER_CM*50);
        //hang();
    }
    public void pos2(){ distance = Math.round(COUNTS_PER_CM*75); }
    public void pos3(){ distance = Math.round(COUNTS_PER_CM*80); }

    public void big(double x){
        arm1.setPosition(x);
        arm2.setPosition(x);
    }

    /*
    public void grab(){
        arm.setPosition(0);
    }

    public void pause(){
        arm.setPosition(0.25);
    }
    public void hang(){
        arm.setPosition(0.45);
    }*/

    //public void transfer(){ arm.setPosition(0.5);}

    public void out(double val){
        swing.setPosition(-val+1);
    }

    public void transfer(){
        arm1.setPosition(0.23);
        arm2.setPosition(0.23);
        swing.setPosition(0);
    }
    public void front(){
        arm1.setPosition(0.60);
        arm2.setPosition(0.60);
        swing.setPosition(0.3);
    }

    public void offwall(){
        arm1.setPosition(0.80);
        arm2.setPosition(0.80);
        swing.setPosition(0.2);
    }

    public void scorespec(){
        arm1.setPosition(0.60);
        arm2.setPosition(0.60);
        swing.setPosition(0.3);
    }
    public void keepPosExceptArms(double pos){
        swing.setPosition(0);
        arm1.setPosition(0);
        arm2.setPosition(0);
    }
    public double addArmPos(double pos){
        double armPos = arm1.getPosition();
        armPos += pos;
        armPos = Math.min(1, Math.max(0, armPos));
        arm1.setPosition(armPos);
        arm2.setPosition(armPos);
        return armPos;
    }
    public double addSwingPos(double pos){
        double swingPos = swing.getPosition();
        swingPos += pos;
        swingPos = Math.min(1, Math.max(0, swingPos));
        swing.setPosition(swingPos);
        return swingPos;
    }


    public void openClaw(){ claw.setPosition(1); }
    public void closeClaw(){ claw.setPosition(0); }
    public void updatePID(){
        ref = (slide1.getCurrentPosition() + slide2.getCurrentPosition()) >> 1;
        double power = PID(distance,ref);
        slide1.setPower(power);
        slide2.setPower(power);
    }

    public double PID(double refrence, double state) {
        double error = refrence - state;
        integralSum += error * timer.seconds(); //
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }
}
