package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.hardware.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.motion.MotionProfileController;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import static org.firstinspires.ftc.teamcode.utils.control.ConfigVariables.UpperSlideVars;

public class UpperSlide extends SubsystemBase {
    // Hardware components
    public ServoImplEx arm1;
    public ServoImplEx arm2;
    public ServoImplEx swing;
    public ServoImplEx claw;
    private DcMotorEx slide1, slide2;
    public ServoImplEx extendo;

    PwmControl.PwmRange v4range = new PwmControl.PwmRange(500, 2500);

    // Control ranges
    private final PwmControl.PwmRange swingRange = new PwmControl.PwmRange(500, 2500);
    private final PwmControl.PwmRange armRange = new PwmControl.PwmRange(500, 2500);
    private final PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 1270);
    private final PwmControl.PwmRange extendoRange = new PwmControl.PwmRange(500, 1270);

    // Position control
    public final PIDFController pidfController;
    public final MotionProfileController motionProfileController;
    private boolean motionProfileEnabled = false;
    public int tickOffset = 0;

    // Constants for encoder calculations
    private static final double PI = 3.14;
    private static final double COUNTS_PER_MOTOR_REV = 28.0;
    private static final double WHEEL_CIRCUMFERENCE_MM = 34 * PI;
    private static final double DRIVE_GEAR_REDUCTION = 5.23;
    private static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private static final double COUNTS_PER_CM = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM) * 10;

    public UpperSlide() {
        super("upperslide");
        pidfController = new PIDFController(
                UpperSlideVars.PID_KP,
                UpperSlideVars.PID_KI,
                UpperSlideVars.PID_KD,
                UpperSlideVars.PID_KF);

        // Initialize motion profile controller with the PIDF controller
        motionProfileController = new MotionProfileController(
                pidfController,
                UpperSlideVars.MAX_VELOCITY_CM_S * COUNTS_PER_CM, // Convert cm/s to ticks/s
                UpperSlideVars.MAX_ACCELERATION_CM_S2 * COUNTS_PER_CM, // Convert cm/s² to ticks/s²
                UpperSlideVars.VELOCITY_FEEDFORWARD,
                UpperSlideVars.ACCELERATION_FEEDFORWARD);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        // Initialize slide motors for power
        slide1 = hardwareMap.get(DcMotorEx.class, ExpansionHub.motor(0));
        slide2 = hardwareMap.get(DcMotorEx.class, ExpansionHub.motor(3));

        // Initialize servos
        arm1 = hardwareMap.get(ServoImplEx.class, ControlHub.servo(2));
        arm2 = hardwareMap.get(ServoImplEx.class, ExpansionHub.servo(1));
        swing = hardwareMap.get(ServoImplEx.class, ControlHub.servo(1));
        claw = hardwareMap.get(ServoImplEx.class, ControlHub.servo(3));
        extendo = hardwareMap.get(ServoImplEx.class, ControlHub.servo(4));

        // Configure directions
        slide2.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.FORWARD);

        arm1.setDirection(ServoImplEx.Direction.FORWARD);
        arm2.setDirection(ServoImplEx.Direction.REVERSE);
        swing.setDirection(ServoImplEx.Direction.FORWARD);

        // Configure servo ranges
        arm1.setPwmRange(armRange);
        arm2.setPwmRange(armRange);
        swing.setPwmRange(swingRange);
        claw.setPwmRange(clawRange);
        extendo.setPwmRange(extendoRange);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Configure motor modes
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set current alarm
        slide1.setCurrentAlert(6.0, CurrentUnit.AMPS);
        slide2.setCurrentAlert(6.0, CurrentUnit.AMPS);
    }

    @Override
    public void periodic(TelemetryPacket packet) {
        // PERFORMANCE OPTIMIZATION: Reduced telemetry frequency
        // Only update essential telemetry to reduce I/O overhead
        double currentPos = getCurrentPosition();
        packet.put("upperslide/position", currentPos);
        packet.put("upperslide/target", pidfController.destination);
        packet.put("upperslide/error", currentPos - pidfController.destination);

        // PERFORMANCE OPTIMIZATION: Servo position reads and current monitoring are
        // expensive
        // Comment out non-essential telemetry to improve loop times
        // packet.put("upperslide/position2", slide2.getCurrentPosition() + tickOffset);
        // packet.put("upperslide/arm1", arm1.getPosition());
        // packet.put("upperslide/arm2", arm2.getPosition());
        // packet.put("upperslide/swing", swing.getPosition());
        // packet.put("upperslide/claw", claw.getPosition());
        // packet.put("upperslide/extendo", extendo.getPosition());

        // PERFORMANCE OPTIMIZATION: Current monitoring is very expensive I2C operation
        // Only enable for debugging motor issues
        // packet.put("upperslide/current1", slide1.getCurrent(CurrentUnit.AMPS));
        // packet.put("upperslide/current2", slide2.getCurrent(CurrentUnit.AMPS));
    }

    /**
     * Set slide position in centimeters
     */
    public void setPositionCM(double cm) {
        pidfController.setDestination(Math.round(COUNTS_PER_CM * cm));
    }

    public void setTickOffset(int tickOffset) {
        this.tickOffset = tickOffset;
    }

    // Preset positions
    public void pos0() {
        setPositionCM(UpperSlideVars.POS_PRE_0_CM);
        setPositionCM(UpperSlideVars.POS_0_CM);
    }

    public void pos1() {
        setPositionCM(UpperSlideVars.POS_1_CM);
    }

    public void pos2() {
        setPositionCM(UpperSlideVars.POS_2_CM);
    }

    public void pos3() {
        setPositionCM(UpperSlideVars.POS_3_CM);
    }

    // Arm position controls
    public void setArmPosition(double position) {
        arm1.setPosition(position);
        arm2.setPosition(position);
    }

    public void setSwingPosition(double position) {
        swing.setPosition(position);
    }

    // Preset arm positions
    public void transfer() {
        setArmPosition(UpperSlideVars.BEHIND_ARM_POS);
        setSwingPosition(UpperSlideVars.BEHIND_SWING_POS);
    }

    public void front() {
        setArmPosition(UpperSlideVars.FRONT_ARM_POS);
        setSwingPosition(UpperSlideVars.FRONT_SWING_POS);
    }

    public void offwall() {
        setArmPosition(UpperSlideVars.OFFWALL_FRONT_ARM_POS);
        setSwingPosition(UpperSlideVars.OFFWALL_FRONT_SWING_POS);
    }

    public void scorespec() {
        setArmPosition(UpperSlideVars.SCORESPEC_FRONT_ARM_POS);
        setSwingPosition(UpperSlideVars.SCORESPEC_FRONT_SWING_POS);
    }

    public void inter() {
        setArmPosition(UpperSlideVars.INTER_ARM_POS);
        setSwingPosition(UpperSlideVars.INTER_SWING_POS);
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

    // Claw controls
    public void openClaw() {
        claw.setPosition(UpperSlideVars.CLAW_OPEN);
    }

    public void closeClaw() {
        claw.setPosition(UpperSlideVars.CLAW_CLOSE);
    }

    // Exentdo controls
    public void openExtendoClaw() {
        extendo.setPosition(UpperSlideVars.EXTENDO_OPEN);
    }

    public void closeExtendoClaw() {
        extendo.setPosition(UpperSlideVars.EXTENDO_CLOSE);
    }

    /**
     * Set slide position in centimeters with motion profiling
     */
    public void setPositionCMWithProfile(double cm) {
        motionProfileEnabled = true;
        double targetTicks = Math.round(COUNTS_PER_CM * cm);
        motionProfileController.setTarget(getCurrentPosition(), targetTicks);
    }

    /**
     * Update motion profile or PID control and return the calculated power
     */
    public double updatePID() {
        double currentPosition = getCurrentPosition();
        double power;

        if (motionProfileEnabled && motionProfileController.isProfileActive()) {
            // Use motion profile control
            power = motionProfileController.calculate(currentPosition);
        } else {
            // Fall back to regular PID control
            motionProfileEnabled = false;
            power = pidfController.calculate(currentPosition);
        }

        // if (pidfController.destination == 0) {
        // // check if current is 0, if so then it means slides have reached bottom, so
        // we
        // // reset encoders to prevent it from going negative
        // if(slide1.isOverCurrent() || slide2.isOverCurrent()) {
        // slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // }
        // }

        slide1.setPower(power);
        slide2.setPower(power);
        return power;
    }

    public void setMotionProfileEnabled(boolean enabled) {
        this.motionProfileEnabled = enabled;
        if (!enabled) {
            motionProfileController.stopProfile();
        }
    }

    /**
     * Check if motion profile is currently active
     */
    public boolean isMotionProfileActive() {
        return motionProfileEnabled && motionProfileController.isProfileActive();
    }

    /**
     * Get current motion profile target position
     */
    public double getMotionProfileTarget() {
        if (isMotionProfileActive()) {
            return motionProfileController.getCurrentTarget();
        }
        return Double.NaN;
    }

    /**
     * Get estimated time remaining for current motion profile
     */
    public double getMotionProfileTimeRemaining() {
        return motionProfileController.getTimeRemaining();
    }

    @Override
    public void stop() {
        // Stop slide motors
        slide1.setPower(0);
        slide2.setPower(0);

        // Disable PIDF control by setting destination to current position
        pidfController.setDestination(getCurrentPosition());

        // Move servos to safe positions
        offwall();
    }

    /**
     * Get the current position of the slide (average of both encoders)
     */
    public double getCurrentPosition() {
        return slide1.getCurrentPosition() + tickOffset;
    }
}
