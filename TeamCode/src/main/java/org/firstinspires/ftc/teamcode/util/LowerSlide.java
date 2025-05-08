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

public class LowerSlide {

    static final double PI = 3.14;
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    // static final double WHEEL_CIRCUMFERENCE_MM = 40.0 * PI;
    // static final double DRIVE_GEAR_REDUCTION = 18.88;
    static final double WHEEL_CIRCUMFERENCE_MM = 37.0 * PI;
    static final double DRIVE_GEAR_REDUCTION = 4.0;
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double COUNTS_PER_CM = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM) * 10;

    HardwareMap hardwareMap;
    public ServoImplEx part1, part2, part3, spinclaw, claw;

    public DcMotor slideMotor;
    public DcMotor slideEncoder;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);
    PwmControl.PwmRange downrange = new PwmControl.PwmRange(500, 900);

    PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1150);

    public PIDController pidController;

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        // 初始化PID控制器
        pidController = new PIDController(
                LowerSlideVars.PID_KP,
                LowerSlideVars.PID_KI,
                LowerSlideVars.PID_KD
        );

        // Initialize slide motor for power
        slideMotor = hardwareMap.get(DcMotor.class, control.motor(2));
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize slide encoder
        slideEncoder = hardwareMap.get(DcMotor.class, expansion.motor(2));
        slideEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        slideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize servos
        part2 = hardwareMap.get(ServoImplEx.class, control.servo(0));
        claw = hardwareMap.get(ServoImplEx.class, expansion.servo(0));
        spinclaw = hardwareMap.get(ServoImplEx.class, expansion.servo(2));
        part1 = hardwareMap.get(ServoImplEx.class, expansion.servo(3));

        part1.setDirection(Servo.Direction.FORWARD);
        part1.setPwmRange(v4range);
        part2.setDirection(Servo.Direction.FORWARD);
        part2.setPwmRange(v4range);

        spinclaw.setPwmRange(v4range);
        claw.setPwmRange(clawRange);
    }

    public void low(double val) {
        slideMotor.setTargetPosition((int) val);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSlidePower(double power) {
        slideMotor.setPower(power);
    }

    public void keepPosExceptArms(double pos) {
        part1.setPosition(0);
        part2.setPosition(0);
    }

    public void setPositionCM(double cm) {
        pidController.setDestination(COUNTS_PER_CM * cm);
    }

    public void big(double val) {
        part1.setPosition(val);
    }

    public void small(double val) {
        part2.setPosition(val);
    }

    public void smallclaw(double val) {
        part3.setPosition(val);
    }

    public void spinclawSetPositionDeg(double degree) {
        spinclaw.setPosition(degree / 270);
    }

    public void pos4() {
        spinclaw.setPosition(0);
    }
    public void posNow() {
        pidController.setDestination(slideEncoder.getCurrentPosition());
    }

    public void pos_grab() {
        big(LowerSlideVars.GRAB_BIG);
        small(LowerSlideVars.GRAB_SMALL);
    }

    public void pos_up() {
        big(LowerSlideVars.UP_BIG);
        small(LowerSlideVars.UP_SMALL);
    }

    public void pos_hover() {
        big(LowerSlideVars.HOVER_BIG);
        small(LowerSlideVars.HOVER_SMALL);
    }

    public void setSlidePos1() {
        pidController.setDestination(Math.round(COUNTS_PER_CM * 50));
    }

    public void setSlidePos2() {
        pidController.setDestination(0);
    }

    public double updatePID() {
        double power = pidController.calculate(slideEncoder.getCurrentPosition());
        slideMotor.setPower(power);
        return power;
    }

    public void closeClaw() {
        claw.setPosition(LowerSlideVars.CLAW_CLOSE);
    }

    public void openClaw() {
        claw.setPosition(LowerSlideVars.CLAW_OPEN);
    }

}
