package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import static org.firstinspires.ftc.teamcode.utils.control.ConfigVariables.LowerSlideVars;

public class LowerSlide extends SubsystemBase {
    // Hardware components
    private ServoImplEx part1, part2, spinclaw, claw;
    public DcMotor slideMotor;

    // Control ranges
    private final PwmControl.PwmRange servoRange = new PwmControl.PwmRange(500, 2500);
    private final PwmControl.PwmRange clawRange = new PwmControl.PwmRange(500, 2500);

    // Position control
    public final PIDFController pidfController;
    private boolean PIDEnabled = true;
    public int tickOffset = 0;

    // Constants for encoder calculations
    private static final double PI = 3.14;
    private static final double COUNTS_PER_MOTOR_REV = 28.0;
    private static final double WHEEL_CIRCUMFERENCE_MM = 37.0 * PI;
    private static final double DRIVE_GEAR_REDUCTION = 3.5;
    private static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    private static final double COUNTS_PER_CM = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM) * 10;

    public LowerSlide() {
        super("lowerslide");
        pidfController = new PIDFController(
                LowerSlideVars.PID_KP,
                LowerSlideVars.PID_KI,
                LowerSlideVars.PID_KD,
                0);
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        // Initialize slide motor and encoder
        slideMotor = hardwareMap.get(DcMotor.class, ExpansionHub.motor(2));

        // Configure motor direction and mode
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        part2 = hardwareMap.get(ServoImplEx.class, ControlHub.servo(0));
        claw = hardwareMap.get(ServoImplEx.class, ExpansionHub.servo(0));
        spinclaw = hardwareMap.get(ServoImplEx.class, ExpansionHub.servo(2));
        part1 = hardwareMap.get(ServoImplEx.class, ExpansionHub.servo(3));

        // Configure servo directions
        part1.setDirection(ServoImplEx.Direction.FORWARD);
        part2.setDirection(ServoImplEx.Direction.FORWARD);

        // Configure servo ranges
        part1.setPwmRange(servoRange);
        part2.setPwmRange(servoRange);
        spinclaw.setPwmRange(servoRange);
        claw.setPwmRange(clawRange);
    }

    public void low(double val) {
        slideMotor.setTargetPosition((int) val);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void keepPosExceptArms(double pos) {
        part1.setPosition(pos);
        part2.setPosition(pos);
    }

    @Override
    public void periodic(TelemetryPacket packet) {
        // Add slide positions to telemetry
        packet.put("lowerslide/position", getCurrentPosition());
        packet.put("lowerslide/target", pidfController.destination);
        packet.put("lowerslide/error", getCurrentPosition() - pidfController.destination);

        // Add servo positions to telemetry
        packet.put("lowerslide/part1", part1.getPosition());
        packet.put("lowerslide/part2", part2.getPosition());
        packet.put("lowerslide/spinclaw", spinclaw.getPosition());
        packet.put("lowerslide/claw", claw.getPosition());
    }

    /**
     * Set slide position in centimeters
     */
    public void setPositionCM(double cm) {
        pidfController.setDestination(Math.round(COUNTS_PER_CM * cm));
    }

    /**
     * Set the slide motor power directly
     */
    public void setSlidePower(double power) {
        slideMotor.setPower(power);
    }

    /**
     * Hold current position
     */
    public void posNow() {
        pidfController.setDestination(getCurrentPosition());
    }

    /**
     * Set part1 servo position
     */
    public void setPart1Position(double position) {
        part1.setPosition(position);
    }

    /**
     * Set part2 servo position
     */
    public void setPart2Position(double position) {
        part2.setPosition(position);
    }

    /**
     * Set spinclaw position in degrees (0-270)
     */
    public void spinclawSetPositionDeg(double degree) {
        spinclaw.setPosition(degree / 270);
    }

    // Preset positions for grabbing action
    public void pos_grab() {
        setPart1Position(LowerSlideVars.GRAB_BIG);
        setPart2Position(LowerSlideVars.GRAB_SMALL);
    }

    public void pos_up() {
        setPart1Position(LowerSlideVars.UP_BIG);
        setPart2Position(LowerSlideVars.UP_SMALL);
    }

    public void pos_hover() {
        setPart1Position(LowerSlideVars.HOVER_BIG);
        setPart2Position(LowerSlideVars.HOVER_SMALL);
    }

    // Preset slide positions

    public void setSlidePos0() {
        setPositionCM(LowerSlideVars.POS_0_CM);
    }

    public void setSlidePos1() {
        setPositionCM(LowerSlideVars.POS_1_CM);
    }

    public void setSlidePos2() {
        setPositionCM(LowerSlideVars.POS_2_CM);
    }
    public void setTickOffset(int tickOffset) { this.tickOffset = tickOffset; }

    // Claw controls
    public void closeClaw() {
        claw.setPosition(LowerSlideVars.CLAW_CLOSE);
    }

    public void openClaw() {
        claw.setPosition(LowerSlideVars.CLAW_OPEN);
    }

    /**
     * Update PID control and return the calculated power
     */
    public double updatePID() {
        if(!PIDEnabled) return 0;
        double power = pidfController.calculate(getCurrentPosition());
        slideMotor.setPower(power);
        return power;
    }

     public void setPIDEnabled(boolean enabled) {
     this.PIDEnabled = enabled;
     }

    @Override
    public void stop() {
        // Stop slide motor
        slideMotor.setPower(0);

        // Move servos to safe positions
        setPart1Position(LowerSlideVars.UP_BIG);
        setPart2Position(LowerSlideVars.UP_SMALL);
        spinclawSetPositionDeg(LowerSlideVars.ZERO + 45);
    }

    /**
     * Get the current position of the slide
     */
    public double getCurrentPosition() {
        return slideMotor.getCurrentPosition() + tickOffset;
    }

    public double getCurrentPositionCM() {
        return getCurrentPosition() / COUNTS_PER_CM;
    }
}
