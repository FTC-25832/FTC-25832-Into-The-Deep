package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.util.ConfigVariables.LowerSlideVars;

public class LowerSlide {

    // double Kp = PIDConstant.Kp;
    double Kp = 0.02;
    double Ki = PIDConstant.Ki;
    double Kd = PIDConstant.Kd;
    double lastError;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0;
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

    public double distance = 0;
    public double ref = 0;

    public void initialize(HardwareMap map) {
        hardwareMap = map;

        // Initialize slide motor for power
        slideMotor = hardwareMap.get(DcMotor.class, control.motor(2));
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize slide encoder
        slideEncoder = hardwareMap.get(DcMotor.class, expansion.motor(1));

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

    public void keepPosExceptArms(double pos) {
        part1.setPosition(0);
        part2.setPosition(0);
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

    // Adjusts slide extension based on camera angle to reach detected objects
    public void autoAdjustExtension(double cameraAngle) {
        // Convert angle to radians for trig calculations
        double angleRadians = Math.toRadians(cameraAngle - ConfigVariables.General.CLAW_ZERO_DEG);

        // Assuming the camera is mounted at a fixed height and angle:
        // Using basic trigonometry to calculate required extension
        // tan(θ) = opposite/adjacent
        // where opposite is the height difference and adjacent is the horizontal
        // distance

        // Base height of the slide mechanism (adjust as needed)
        final double BASE_HEIGHT_CM = 30.0;
        // Target height where the camera is detecting the object
        double targetDistance = BASE_HEIGHT_CM / Math.tan(angleRadians);

        // Convert the calculated distance to motor counts
        distance = Math.round(COUNTS_PER_CM * targetDistance);

        // Ensure the distance is within safe limits
        distance = Math.min(Math.max(distance, 0),
                Math.round(COUNTS_PER_CM * LowerSlideVars.POS_2_CM));

        // Use hover position for the servos
        pos_hover();
    }

    public void setSlidePos1() {
        distance = Math.round(COUNTS_PER_CM * LowerSlideVars.POS_1_CM);
    }

    public void setSlidePos2() {
        distance = Math.round(COUNTS_PER_CM * LowerSlideVars.POS_2_CM);
    }

    public void closeClaw() {
        claw.setPosition(LowerSlideVars.CLAW_CLOSE);
    }

    public void openClaw() {
        claw.setPosition(LowerSlideVars.CLAW_OPEN);
    }

    public void updatePID() {
        ref = slideEncoder.getCurrentPosition();
        double power = PID(distance, ref);
        slideMotor.setPower(power);
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
