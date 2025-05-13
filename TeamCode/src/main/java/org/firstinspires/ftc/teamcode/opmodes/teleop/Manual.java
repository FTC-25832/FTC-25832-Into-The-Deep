package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

//import java.io.ObjectInputFilter.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import com.bylazar.ftcontrol.panels.Panels;
//import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
//import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
//import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
//import com.bylazar.ftcontrol.panels.json.Circle;
//import com.bylazar.ftcontrol.panels.json.Line;
//import com.bylazar.ftcontrol.panels.json.Look;
//import com.bylazar.ftcontrol.panels.json.Point;
//import com.bylazar.ftcontrol.panels.json.Rectangle;

import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
//import org.firstinspires.ftc.teamcode.util.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.timing.Timeout;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

@TeleOp(group = "TeleOp")
public class Manual extends LinearOpMode {
    // Localizer odo = new Localizer();
    final double BUTTONPRESSINTERVALMS = 80;

    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    Hanging hangingServos = new Hanging();
    // Limelight camera = new Limelight();
    PIDFController PIDY = new PIDFController(
            ConfigVariables.Camera.PID_KP,
            ConfigVariables.Camera.PID_KI,
            ConfigVariables.Camera.PID_KD,
            ConfigVariables.Camera.PID_KF);
    IMU imu;

    double lastTimeGP1LeftBumperCalled = 0;
    double lastTimeGP2LeftBumperCalled = 0;
    boolean upClawIsOpen = false;
    boolean lowClawIsOpen = false;

    Canvas field;

    // private TelemetryManager ftControlTelemetry;
    private final long lastDashboardUpdateTime = 0;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // private Panels panels;
        // private Panels ftControlDashboard;
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        // panelsT = panels.getTelemetry();
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

        upslide.initialize(hardwareMap);
        lowslide.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        hangingServos.initialize(hardwareMap);
        // camera.initialize(hardwareMap);

        upslide.keepPosExceptArms(0);
        lowslide.keepPosExceptArms(0);

        upslide.front();
        lowslide.pos_up();
        waitForStart();

        // camera.cameraStart();

        while (opModeIsActive()) {

            controlDrivetrain();
            controlUpslide();
            controlLowslide();
            controlHanging();
            // upslide.big(gamepad1.right_trigger);
            // upslide.swing.setPosition(gamepad1.left_trigger);

            // lowslide.big(-gamepad2.left_stick_y);
            // lowslide.small(-gamepad2.right_stick_y);
            // lowslide.spinclaw.setPosition(gamepad2.right_trigger);

            // if(gamepad2.left_bumper){ upslide.closeClaw(); }
            // if(gamepad2.right_bumper){ upslide.openClaw(); }
            // if(gamepad1.left_bumper){ lowslide.closeClaw(); }
            // if(gamepad1.right_bumper){ lowslide.openClaw(); }

            double time = System.currentTimeMillis();
            if (gamepad1.left_bumper) {
                if (time - lastTimeGP1LeftBumperCalled > BUTTONPRESSINTERVALMS) {
                    lowClawIsOpen = !lowClawIsOpen;
                }
                lastTimeGP1LeftBumperCalled = time;
            }
            if (gamepad2.left_bumper) {
                if (time - lastTimeGP2LeftBumperCalled > BUTTONPRESSINTERVALMS) {
                    upClawIsOpen = !upClawIsOpen;
                }
                lastTimeGP2LeftBumperCalled = time;
            }
            if (lowClawIsOpen)
                lowslide.openClaw();
            else
                lowslide.closeClaw();
            if (upClawIsOpen)
                upslide.openClaw();
            else
                upslide.closeClaw();

            double upslidePower = upslide.updatePID();
            double lowslidePower = lowslide.updatePID();
            telemetry.addData("upslide power", upslidePower);
            telemetry.addData("upslide destination", upslide.pidfController.destination);
            telemetry.addData("upslide position", upslide.getCurrentPosition());
            telemetry.addData("lowslide power", lowslidePower);
            telemetry.addData("lowslide destination", lowslide.pidController.destination);
            telemetry.addData("lowslide position", lowslide.getCurrentPosition());
            // Create dashboard packet
            // TelemetryPacket packet = new TelemetryPacket();
            field = packet.fieldOverlay();

            // Draw robot position from odometry
            field.setStroke("#3F51B5"); // Material Blue
            DashboardUtil.drawRobot(field, "#3F51B5"); // Draw robot using localizer data
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
            // panels.debug("Loop ${System.currentTimeMillis()} ran!");
            // panels.update();
        }
    }

    boolean grabTimeoutset = false;

    private void setGrabSequence() {
        if (grabTimeoutset)
            return;
        lowslide.openClaw();
        new Timeout(() -> lowslide.pos_grab(), ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT);
        new Timeout(() -> lowslide.closeClaw(), ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT);
        new Timeout(() -> {
            lowslide.pos_hover();
            grabTimeoutset = false;
        }, ConfigVariables.LowerSlideVars.POS_HOVER_TIMEOUT);
        grabTimeoutset = true;
    }

    public void controlHanging() {
        if (gamepad2.right_trigger > 0) {
            hangingServos.turnForward();
        }
        if (gamepad2.left_trigger > 0) {
            hangingServos.turnBackward();
        }
        if (gamepad2.right_bumper) {
            hangingServos.stop();
        }
    }

    private void controlDrivetrain() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // double botHeading = odo.heading();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1; // Counteract imperfect strafing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive.setFrontLeft(frontLeftPower);
        drive.setBackLeft(backLeftPower);
        drive.setFrontRight(frontRightPower);
        drive.setBackRight(backRightPower);

        telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Heading", botHeading);
    }

    private void controlUpslide() {
        if (gamepad2.a) {
            upslide.pos0();
        }
        if (gamepad2.x) {
            upslide.pos1();
        }
        if (gamepad2.y) {
            upslide.pos2();
        }
        if (gamepad2.b) {
            upslide.pos3();
        }
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

    private void controlLowslide() {
        if (gamepad1.right_bumper) {
            lowslide.pos_hover();
        }
        if (gamepad1.right_trigger > 0) {
            setGrabSequence();
        }
        if (gamepad1.left_trigger > 0) {
            lowslide.pos_up();
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.SPINCLAW_DEG);
        }
        if (gamepad1.x) {
            lowslide.setSlidePos1();
        }
        if (gamepad1.y) {
            lowslide.setSlidePos2();
        }
        if (gamepad1.dpad_down) {
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO);
        }
        if (gamepad1.dpad_right) {
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO + 45);
        }
        if (gamepad1.dpad_up) {
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO + 90);
        }
    }
}
