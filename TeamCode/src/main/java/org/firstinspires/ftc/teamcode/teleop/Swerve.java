package org.firstinspires.ftc.teamcode.teleop;

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

import org.firstinspires.ftc.teamcode.util.ConfigVariables;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Hanging;
import org.firstinspires.ftc.teamcode.util.Limelight;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Timeout;
import org.firstinspires.ftc.teamcode.util.UpperSlide;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {
    // Localizer odo = new Localizer();
    final double BUTTONPRESSINTERVALMS = 80;

    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    Limelight camera = new Limelight();
    Hanging hangingServos = new Hanging();
    PIDController PIDY = new PIDController(
            ConfigVariables.Camera.PID_KP,
            ConfigVariables.Camera.PID_KI,
            ConfigVariables.Camera.PID_KD,
            ConfigVariables.Camera.PID_KF);
    IMU imu;

    boolean adjust = false;
    double lastTimeGP1LeftBumperCalled = 0;
    double lastTimeGP2LeftBumperCalled = 0;
    boolean upClawIsOpen = false;
    boolean lowClawIsOpen = false;

    Canvas field;

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
        camera.initialize(hardwareMap);
        camera.cameraStart();

        upslide.keepPosExceptArms(0);
        lowslide.keepPosExceptArms(0);
        PIDY.setDestination(0);

        upslide.front();
        lowslide.pos_up();
        waitForStart();

        camera.cameraStart();

        while (opModeIsActive()) {
            if (adjust) {
                adjustIntake();
                adjust = false;
                // sleep(1000);
                // lowslide.openClaw();
                // sleep(200);
                // lowslide.pos_grab();
                // sleep(400);
                // lowslide.closeClaw();
                // sleep(400);
                // lowslide.pos_up();
                // sleep(400);
            }

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
//            bool adjustBackTimeoutSet = true;
            if (lowClawIsOpen) {
                lowslide.openClaw();
                lowClawIsOpen = false;
                lowslide.pos_grab();
//                adjustBackTimeoutSet = false;
//                if (!adjustBackTimeoutSet) {
//
//                    adjustBackTimeoutSet = true;
//                }
                new Timeout(() -> lowslide.closeClaw(), 500);
                lowslide.pos_hover();
            } else
                lowslide.closeClaw();
            if (upClawIsOpen)
                upslide.openClaw();
            else
                upslide.closeClaw();

            double upslidePower = upslide.updatePID();
            double lowslidePower = lowslide.updatePID();

            // Upslide PID Telemetry
            telemetry.addData("upslide power", upslidePower);
            telemetry.addData("upslide target", upslide.pidController.destination);
            telemetry.addData("upslide position", upslide.slide1Encoder.getCurrentPosition());
            telemetry.addData("upslide error",
                    upslide.slide1Encoder.getCurrentPosition() - upslide.pidController.destination);

            // Lowslide Telemetry
            telemetry.addData("lowslide power", lowslidePower);
            telemetry.addData("lowslide destination", lowslide.pidController.destination);
            telemetry.addData("lowslide position", lowslide.slideEncoder.getCurrentPosition());

            // Dashboard Telemetry for PID Tuning
            packet.put("upslide/power", upslidePower); // Group upslide data
            packet.put("upslide/target", upslide.pidController.destination);
            packet.put("upslide/position", upslide.slide1Encoder.getCurrentPosition());
            packet.put("upslide/error", upslide.slide1Encoder.getCurrentPosition() - upslide.pidController.destination);

            // Other telemetry
            packet.put("lowslide power", lowslidePower);
            packet.put("lowslide destination", lowslide.pidController.destination);
            packet.put("lowslide position", lowslide.slideEncoder.getCurrentPosition());

            // Create dashboard packet
            // TelemetryPacket packet = new TelemetryPacket();
            field = packet.fieldOverlay();

            // Draw robot position from odometry
            field.setStroke("#3F51B5"); // Material Blue
            DashboardUtil.drawRobot(field, "#3F51B5"); // Draw robot using localizer data
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
            dashboardTelemetry.update();
            // panels.debug("Loop ${System.currentTimeMillis()} ran!");
            // panels.update();
        }
    }

    boolean isAdjustTimeout = false;
    boolean isAngleTimeout = false;
    boolean pidUpdated = false;

    private void adjustIntake() {
        boolean adjustBackTimeoutSet = false;
        PIDY.reset();
        if (!camera.updateDetectorResult()) {
            gamepad1.rumble(100);
            return;
        }
        // one-time adjustment
        new Timeout(() -> isAdjustTimeout = true, ConfigVariables.Camera.ADJUST_TIMEOUT);
        double dyAccum = 0;
        int dyNum = 1;
        while (!isAdjustTimeout) {
            camera.updateDetectorResult();
            // processing position
            double dy = camera.getY();
            if (dy == 0) {
                continue;
            }
            dyAccum += dy;
            dyNum += 1;
        }
        double averageY = dyAccum / dyNum;
        averageY = Math.max(0, Math.min(averageY, ConfigVariables.Camera.DISTANCE_MAP.length - 1));
        double position = ConfigVariables.Camera.DISTANCE_MAP[(int) Math.floor(averageY)]
                + (averageY - Math.floor(averageY)) *
                        (ConfigVariables.Camera.DISTANCE_MAP[(int) Math.floor(averageY) + 1]
                                - ConfigVariables.Camera.DISTANCE_MAP[(int) Math.floor(averageY)]);
        lowslide.setPositionCM(position - ConfigVariables.Camera.CLAW_DISTANCE);

        while (!pidUpdated) {
            lowslide.updatePID();
            if (lowslide.pidController.destination - lowslide.pidController.pos > Math
                    .abs(ConfigVariables.Camera.DISTANCE_THRESHOLD)) {
                new Timeout(() -> pidUpdated = true, ConfigVariables.Camera.PID_UPDATE_TIMEOUT);
            }
        }

        // keep adjusting
        boolean isAdjusted = false;
        while (!isAdjusted) {
            camera.updateDetectorResult();
            // processing position
            double dy = camera.getY();
            dy = Math.max(0, dy);
            double ypower = PIDY.calculate(-dy); // input is the position now
            lowslide.setSlidePower(ypower);
            controlDrivetrain();
            if (gamepad1.right_trigger > 0.5) {
                isAdjusted = true;
            }
            // if (Math.abs(dy) < ConfigVariables.Camera.DISTANCE_THRESHOLD){
            // if(!adjustBackTimeoutSet){
            // new Timeout(()->isAdjusted=true, ConfigVariables.Camera.ADJUST_EXTRA_TIME);
            // adjustBackTimeoutSet = true;
            // }
            // }



            telemetry.addData("adjusting", "true");
            telemetry.addData("Moveto", position);
            telemetry.addData("Y difference", dy);
            telemetry.addData("ypower", ypower);
            telemetry.update();
        }
        lowslide.posNow();
        lowslide.pos_hover();
        // spinclaw adjustment
        new Timeout(() -> isAngleTimeout = true, ConfigVariables.Camera.ANGLE_TIMEOUT);
        camera.switchtoPython();
        camera.setColor(camera.getClassname());
        double angleAccum = 0;
        double angleNum = 1;
        while (!isAngleTimeout) {
            controlUpslide();
            // camera.updateDetectorResult(); // used when using neural detector
            // processing angle for spinclaw
            double angle = camera.getAngle(); // -90 ~ 90
            angle = angle + ConfigVariables.Camera.ANGLE_OFFSET;
            angleAccum += angle;
            angleNum += 1;
            drawAngle(angle);
            telemetry.addData("angle", angle);
            telemetry.update();
        }
        double averageAngle = angleAccum / angleNum;
        lowslide.spinclawSetPositionDeg(averageAngle);
        camera.switchtoNeural();
        camera.reset();
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

        drive.fl(frontLeftPower);
        drive.bl(backLeftPower);
        drive.fr(frontRightPower);
        drive.br(backRightPower);

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

        telemetry.addData("upslide arm", upslide.arm1.getPosition());
        telemetry.addData("upslide swing", upslide.swing.getPosition());
    }

    private void controlLowslide() {
        if (gamepad1.right_bumper) {
            adjust = true;
        }
        if (gamepad1.right_trigger > 0) {
            lowslide.pos_up();
            adjust = false;
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.SPINCLAW_DEG);
        }
        if (gamepad1.left_trigger > 0) {
            lowslide.pos_grab();
            adjust = false;
        }
        if (gamepad1.x) {
            lowslide.setSlidePos1();
        }
        if (gamepad1.y) {
            lowslide.setSlidePos2();
        }
        if (gamepad1.dpad_down) {
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO + 45);
        }
        if (gamepad1.dpad_left) {
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO);
        }
        if (gamepad1.dpad_right) {
            lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO + 90);
        }
    }

    private void drawAngle(double angle) {
        // Visualize limelight detection (camera)
        field.setStroke("#4CAF50"); // Material Green for detection
        field.setFill("#4CAF50");
        double radians = Math.toRadians(angle);
        field.strokeLine(0, 0, 20 * Math.cos(radians), 20 * Math.sin(radians));
        field.fillCircle(20 * Math.cos(radians), 20 * Math.sin(radians), 3);
    }
}
