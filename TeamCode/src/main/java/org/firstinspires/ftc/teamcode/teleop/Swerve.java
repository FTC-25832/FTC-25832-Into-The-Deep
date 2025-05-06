package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Limelight;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.util.UpperSlide;
import org.firstinspires.ftc.teamcode.util.Interval;

@TeleOp(group="TeleOp")
public class Swerve extends LinearOpMode {
    static final double ANGLE_OFFSET = 55;
    static final double DISTANCE_THRESHOLD = 10;
    static final int CROSSHAIR_X = 300;
    static final int CROSSHAIR_Y = 300;
    final double BUTTONPRESSINTERVALMS=80;
    Localizer odo = new Localizer();
    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    Limelight camera = new Limelight();
    PIDController PIDX = new PIDController(0.01, 0.0, 0.0);
    PIDController PIDY = new PIDController(0.01, 0.0, 0.0);
    boolean adjust = false;
    double lastTimeGP1LeftBumperCalled=0;
    double lastTimeGP2LeftBumperCalled=0;
    boolean upClawIsOpen=false;
    boolean lowClawIsOpen=false;

    @Override
    public void runOpMode() throws InterruptedException {
        odo.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        upslide.initialize(hardwareMap);
        lowslide.initialize(hardwareMap);
        camera.initialize(hardwareMap);
        camera.cameraStart();

        upslide.keepPosExceptArms(0);
        lowslide.keepPosExceptArms(0);
        PIDX.setDestination(CROSSHAIR_X);
        PIDY.setDestination(CROSSHAIR_Y);

        upslide.front();
        lowslide.pos_up();
        waitForStart();

        camera.cameraStart();

        while (opModeIsActive()) {
            if(adjust){
                adjustIntake();
                adjust = false;
            }

            controlDrivetrain();
            controlUpslide();
            controlLowslide();
//            upslide.big(gamepad1.right_trigger);
//            upslide.swing.setPosition(gamepad1.left_trigger);


//            lowslide.big(-gamepad2.left_stick_y);
//            lowslide.small(-gamepad2.right_stick_y);
//            lowslide.spinclaw.setPosition(gamepad2.right_trigger);

//            if(gamepad2.left_bumper){ upslide.closeClaw(); }
//            if(gamepad2.right_bumper){ upslide.openClaw(); }
//            if(gamepad1.left_bumper){ lowslide.closeClaw(); }
//            if(gamepad1.right_bumper){ lowslide.openClaw(); }

            double time=System.currentTimeMillis();
            if (gamepad1.left_bumper) {
                if (time-lastTimeGP1LeftBumperCalled>BUTTONPRESSINTERVALMS) {
                    lowClawIsOpen = !lowClawIsOpen;
                }
                lastTimeGP1LeftBumperCalled = time;
            }
            if (gamepad2.left_bumper) {
                if (time-lastTimeGP2LeftBumperCalled>BUTTONPRESSINTERVALMS) {
                    upClawIsOpen = !upClawIsOpen;
                }
                lastTimeGP2LeftBumperCalled = time;
            }
            if (lowClawIsOpen) lowslide.openClaw();
            else lowslide.closeClaw();
            if (upClawIsOpen) upslide.openClaw();
            else upslide.closeClaw();

            upslide.updatePID();
            lowslide.updatePID();

            //Telemetry
            //telemetry.addData("claw", upslide.claw.getPosition());

            telemetry.addData("Status", "Running");

//            telemetry.addData("Distance", lowslide.distance);
//            telemetry.addData("State", lowslide.slide.getCurrentPosition());
//            telemetry.addData("Power", lowslide.PID(lowslide.distance, lowslide.slide.getCurrentPosition()));

            telemetry.update();
        }
    }
    double angleAccum = 0;
    double angleNum = 1;
    boolean isAdjustTimeout = false;
    boolean isAdjusted = false;
    private void adjustIntake(){
        isAdjustTimeout = false;
        isAdjusted = false;
        PIDX.reset();
        PIDY.reset();
        Interval interval = new Interval(() -> {
            double posAngle = angleAccum / angleNum;
            posAngle = Math.min(Math.max(posAngle, 0), 270);
            lowslide.spinclawSetPositionDeg(posAngle);
            angleAccum = 0;
            angleNum = 1;
        }, 300);

        new Timeout(() -> {
            isAdjustTimeout = true;
        }, 5000);

        while(!isAdjustTimeout&&!isAdjusted){
            camera.updatePythonOutput();
            // processing angle for spinclaw
            double angle = camera.getAngle(); // -90 ~ 90
            angle = angle + ANGLE_OFFSET;
            angleAccum += angle;
            angleNum += 1;
            // processing position
            double y = camera.getY();
            double ypower = PIDY.calculate(y);

            lowslide.setSlidePower(ypower);
            controlDrivetrain();

            isAdjusted = Math.abs(y - CROSSHAIR_Y) < DISTANCE_THRESHOLD;
            telemetry.addData("adjusting", "true");
            telemetry.addData("angle", angle);
            telemetry.addData("y", y);
            telemetry.addData("ypower", ypower);
            telemetry.addData("angleAccum", angleAccum);
            telemetry.addData("angleNum", angleNum);
            telemetry.update();
        }
        interval.cancel();
    }
    private void controlDrivetrain(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = odo.heading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive.fl(frontLeftPower);
        drive.bl(backLeftPower);
        drive.fr(frontRightPower);
        drive.br(backRightPower);

        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Heading", botHeading);
    }
    private void controlUpslide(){
        if(gamepad2.a){ upslide.pos0(); }
        if(gamepad2.x){ upslide.pos1(); }
        if(gamepad2.y){ upslide.pos2(); }
        if(gamepad2.b){ upslide.pos3(); }
        if(gamepad2.right_trigger > 0){ upslide.behind(); }
        if(gamepad2.left_trigger > 0){ upslide.front(); }
//        if(gamepad2.left_bumper){
//            upslide.closeClaw();
//        }

        telemetry.addData("right", gamepad2.right_trigger);
        telemetry.addData("left", gamepad2.left_trigger);
        telemetry.addData("big arm", upslide.arm1.getPosition());
        telemetry.addData("small arm", upslide.swing.getPosition());
    }

    private void controlLowslide(){
        if(gamepad1.right_bumper){
            lowslide.pos_hover();
            adjust = true;
        }
        if(gamepad1.right_trigger>0) { lowslide.pos_grab(); adjust = false; }
        if (gamepad1.left_trigger>0){ lowslide.pos_up();  adjust = false; lowslide.spinclawSetPositionDeg(0);}
        if (gamepad1.x) { lowslide.setSlidePos1(); }
        if (gamepad1.y) { lowslide.setSlidePos2(); }
        if(gamepad1.dpad_down) { lowslide.spinclawSetPositionDeg(0); }
        if(gamepad1.dpad_right) { lowslide.spinclawSetPositionDeg(45); }
        if(gamepad1.dpad_up) { lowslide.spinclawSetPositionDeg(90); }
    }
}