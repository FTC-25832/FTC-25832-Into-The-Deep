package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Timeout;

import static org.firstinspires.ftc.teamcode.util.ConfigVariables.UpperSlideVars;

import java.sql.Time;

public class UpperSlide {
    HardwareMap hardwareMap;
    public ServoImplEx arm1, arm2, swing, claw;
    PwmControl.PwmRange swingRange = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange armRange = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1270);
    public PIDController pidController;
    static final double PI = 3.14;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double WHEEL_CIRCUMFERENCE_MM = 34 * PI;
    static final double DRIVE_GEAR_REDUCTION = 5.23;
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double COUNTS_PER_CM = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM) * 10;
    public DcMotor slide1, slide2;
    public DcMotor slide1Encoder, slide2Encoder;

    public void initialize(HardwareMap map) {
        hardwareMap = map;
        pidController = new PIDController(
                UpperSlideVars.PID_KP,
                UpperSlideVars.PID_KI,
                UpperSlideVars.PID_KD,
                UpperSlideVars.PID_KF);
        // Initialize slide motors for power
        slide1 = hardwareMap.get(DcMotor.class, control.motor(0));
        slide2 = hardwareMap.get(DcMotor.class, control.motor(3));

        // Initialize slide encoders
        slide1Encoder = hardwareMap.get(DcMotor.class, expansion.motor(0));
        slide2Encoder = hardwareMap.get(DcMotor.class, expansion.motor(3));

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
        slide2Encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to run without encoder since we're using separate encoder ports
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1Encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2Encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void pos0() {
        pidController.setDestination(Math.round(COUNTS_PER_CM * UpperSlideVars.POS_PRE_0_CM));
        new Timeout(() -> pidController.setDestination(Math.round(COUNTS_PER_CM * UpperSlideVars.POS_0_CM)), 500);
    }

    public void pos1() {
        // closeClaw();
        pidController.setDestination(Math.round(COUNTS_PER_CM * UpperSlideVars.POS_1_CM));
        // hang();
    }

    public void pos2() {
        pidController.setDestination(Math.round(COUNTS_PER_CM * UpperSlideVars.POS_2_CM));
    }

    public void pos3() {
        pidController.setDestination(Math.round(COUNTS_PER_CM * UpperSlideVars.POS_3_CM));
    }

    public void big(double x) {
        arm1.setPosition(x);
        arm2.setPosition(x);
    }

    /*
     * public void grab(){
     * arm.setPosition(0);
     * }
     *
     * public void pause(){
     * arm.setPosition(0.25);
     * }
     * public void hang(){
     * arm.setPosition(0.45);
     * }
     */

    // public void transfer(){ arm.setPosition(0.5);}

    public void out(double val) {
        swing.setPosition(-val + 1);
    }

    public void transfer() {
        arm1.setPosition(UpperSlideVars.BEHIND_ARM_POS);
        arm2.setPosition(UpperSlideVars.BEHIND_ARM_POS);
        swing.setPosition(UpperSlideVars.BEHIND_SWING_POS);
    }

    public void front() {
        arm1.setPosition(UpperSlideVars.FRONT_ARM_POS);
        arm2.setPosition(UpperSlideVars.FRONT_ARM_POS);
        swing.setPosition(UpperSlideVars.FRONT_SWING_POS);
    }

    public void offwall() {
        arm1.setPosition(UpperSlideVars.OFFWALL_FRONT_ARM_POS);
        arm2.setPosition(UpperSlideVars.OFFWALL_FRONT_ARM_POS);
        swing.setPosition(UpperSlideVars.OFFWALL_FRONT_SWING_POS);
    }

    public void scorespec() {
        arm1.setPosition(UpperSlideVars.SCORESPEC_FRONT_ARM_POS);
        arm2.setPosition(UpperSlideVars.SCORESPEC_FRONT_ARM_POS);
        swing.setPosition(UpperSlideVars.SCORESPEC_FRONT_SWING_POS);
    }

    public void keepPosExceptArms(double pos) {
        arm1.setPosition(0);
        arm2.setPosition(0);
        swing.setPosition(0);
    }

    public double addArmPos(double pos) {
        double armPos = arm1.getPosition();
        armPos += pos;
        armPos = Math.min(1, Math.max(0, armPos));
        arm1.setPosition(armPos);
        arm2.setPosition(armPos);
        return armPos;
    }

    public double addSwingPos(double pos) {
        double swingPos = swing.getPosition();
        swingPos += pos;
        swingPos = Math.min(1, Math.max(0, swingPos));
        swing.setPosition(swingPos);
        return swingPos;
    }

    public void openClaw() {
        claw.setPosition(UpperSlideVars.CLAW_OPEN);
    }

    public void closeClaw() {
        claw.setPosition(UpperSlideVars.CLAW_CLOSE);
    }

    public double updatePID() {
        double currentPosition = (slide1Encoder.getCurrentPosition() + slide2Encoder.getCurrentPosition()) / 2.0;
        double power = pidController.calculate(currentPosition) * 0.8; // Power includes feedforward from PIDF

        slide1.setPower(power);
        slide2.setPower(power);
        return power;
    }
}
