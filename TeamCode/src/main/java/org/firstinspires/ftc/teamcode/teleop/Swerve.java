package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.bylazar.ftcontrol.panels.json.Circle;
import com.bylazar.ftcontrol.panels.json.Line;
import com.bylazar.ftcontrol.panels.json.Look;
import com.bylazar.ftcontrol.panels.json.Point;
import com.bylazar.ftcontrol.panels.json.Rectangle;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.ConfigVariables;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Limelight;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.util.UpperSlide;
import org.firstinspires.ftc.teamcode.util.Interval;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {
    // Localizer odo = new Localizer();
    static final double ANGLE_OFFSET = 55;
    static final double DISTANCE_THRESHOLD = 10;
    static final int CROSSHAIR_X = 300;
    static final int CROSSHAIR_Y = 300;
    final double BUTTONPRESSINTERVALMS=80;
    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    Limelight camera = new Limelight();
    PIDController PIDX = new PIDController(0.01, 0.0, 0.0);
    IMU imu;

    boolean adjust = false;
    boolean wasAdjusting = false;
    static final double ANGLE_OFFSET = 145;
    double lastTimeGP1LeftBumperCalled = 0;
    double lastTimeGP2LeftBumperCalled = 0;
    boolean upClawIsOpen = false;
    boolean lowClawIsOpen = false;

    final double buttonPressIntervalMS = 80;

    private FtcDashboard dashboard;
    // private Panels ftControlDashboard;
    private Telemetry dashboardTelemetry;
    // private TelemetryManager ftControlTelemetry;
    private long lastDashboardUpdateTime = 0;
    private static final long DASHBOARD_UPDATE_INTERVAL_MS = 250; // Update FTCdashboard 4 times per second

    TelemetryPacket packet = new TelemetryPacket();

    // dashboards need to be removed in official match, so.
    // Original helper methods for FTC Dashboard
    private void addTelemetryAndPacket(String caption, Object value) {
        telemetry.addData(caption, value);
        if (System.currentTimeMillis() - lastDashboardUpdateTime >= DASHBOARD_UPDATE_INTERVAL_MS) {
            packet.put(caption, value);
        }
    }

    private void addTelemetryAndPacket(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        if (System.currentTimeMillis() - lastDashboardUpdateTime >= DASHBOARD_UPDATE_INTERVAL_MS) {
            packet.put(caption, String.format(format, args));
        }
    }

    // helper for new function
    private void addTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        packet.put(caption, value);
        // ftControlTelemetry.debug(caption + ": " + value);
    }

    private void addTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        packet.put(caption, String.format(format, args));
        // ftControlTelemetry.debug(caption + ": " + String.format(format, args));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
//        panels = Panels.getTelemetry();
        // ftControlDashboard = Panels.getInstance();

        // odo.initialize(hardwareMap);
        /*
         *
         * TEMP DISABLE, DONT NEED THIS ACCURATE ODO FOR HEADING?
         *
         */

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        drive.initialize(hardwareMap);
        upslide.initialize(hardwareMap);
        lowslide.initialize(hardwareMap);
        camera.initialize(hardwareMap);
        camera.cameraStart();

        upslide.keepPosExceptArms(0);
        lowslide.keepPosExceptArms(0);
        PIDX.setDestination(CROSSHAIR_X);

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
                if (time - lastTimeGP1LeftBumperCalled > buttonPressIntervalMS) {
                    lowClawIsOpen = !lowClawIsOpen;
                }
                lastTimeGP1LeftBumperCalled = time;
            }
            if (gamepad2.left_bumper) {
                if (time - lastTimeGP2LeftBumperCalled > buttonPressIntervalMS) {
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


            // Create dashboard packet
            // TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // Draw robot position from odometry
            field.setStroke("#3F51B5"); // Material Blue
            DashboardUtil.drawRobot(field, "#3F51B5"); // Draw robot using localizer data
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
//            panels.debug("Loop ${System.currentTimeMillis()} ran!");
//            panels.update();
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
        Interval interval = new Interval(() -> {
            double posAngle = angleAccum / angleNum;
            posAngle = Math.min(Math.max(posAngle, 0), 270);
            lowslide.spinclawSetPositionDeg(posAngle);
            angleAccum = 0;
            angleNum = 1;
        }, 300);

        // Visualize limelight detection (camera)
        // if (camera.isDetected()) {
        if (camera != null) {
            field.setStroke("#4CAF50"); // Material Green for detection
            field.setFill("#4CAF50");
            double radians = Math.toRadians(angle);
            field.strokeLine(0, 0, 20 * Math.cos(radians), 20 * Math.sin(radians));
            field.fillCircle(20 * Math.cos(radians), 20 * Math.sin(radians), 3);
        }
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

        // double botHeading = odo.heading();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
        if(gamepad2.right_trigger > 0){ upslide.transfer(); }
        if(gamepad2.left_trigger > 0){ upslide.front(); }
        if (gamepad2.dpad_down) {
            upslide.transfer();
        }

        if (gamepad2.dpad_up) {
            upslide.front();
        }
        if (gamepad2.dpad_left) {
            upslide.offwall();
        }
        if (gamepad2.dpad_right) {
            upslide.scorespec();
        }

        telemetry.addData("right", gamepad2.right_trigger);
        telemetry.addData("left", gamepad2.left_trigger);
        telemetry.addData("upslide arm", upslide.arm1.getPosition());
        telemetry.addData("upslide swing", upslide.swing.getPosition());
    }

    private void controlLowslide(){
        if(gamepad1.right_bumper){
            lowslide.pos_hover();
            adjust = true;
        }
        if(gamepad1.right_trigger>0) { lowslide.pos_grab(); adjust = false; }
        if (gamepad1.left_trigger>0){ lowslide.pos_up();  adjust = false; lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.SPINCLAW_DEG);}
        if (gamepad1.x) { lowslide.setSlidePos1(); }
        if (gamepad1.y) { lowslide.setSlidePos2(); }
        if(gamepad1.dpad_down) { lowslide.spinclawSetPositionDeg(0); }
        if(gamepad1.dpad_right) { lowslide.spinclawSetPositionDeg(45); }
        if(gamepad1.dpad_up) { lowslide.spinclawSetPositionDeg(90); }
    }
}
