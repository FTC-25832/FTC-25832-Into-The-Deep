package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

import org.firstinspires.ftc.teamcode.util.ConfigVariables;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Limelight;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LowerSlide;
import org.firstinspires.ftc.teamcode.util.UpperSlide;
import org.firstinspires.ftc.teamcode.util.Interval;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {
    // Localizer odo = new Localizer();
    Drivetrain drive = new Drivetrain();
    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();
    Limelight camera = new Limelight();

    double angleAccum = 0;
    double angleNum = 1;
    boolean adjust = false;
    boolean wasAdjusting = false;
    double lastAngle = 0;
    static final double ANGLE_DIFFERENCE_THRESHOLD = 15.0; // Degrees threshold for rumble
    static final double ANGLE_OFFSET = 145;
    double lastTimeGP1LeftBumperCalled = 0;
    double lastTimeGP2LeftBumperCalled = 0;
    boolean upClawIsOpen = false;
    boolean lowClawIsOpen = false;

    final double buttonPressIntervalMS = 80;

    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;
    private TelemetryManager ftControlTelemetry;
    private long lastDashboardUpdateTime = 0;
    private static final long DASHBOARD_UPDATE_INTERVAL_MS = 250; // Update FTCdashboard 4 times per second

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
        ftControlTelemetry.debug(caption + ": " + value);
    }

    private void addTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        ftControlTelemetry.debug(caption + ": " + String.format(format, args));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        ftControlTelemetry = Panels.getTelemetry();

        // Initialize camera stream
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

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

        upslide.front();
        lowslide.pos_up();
        waitForStart();

        camera.cameraStart();
        dashboard.startCameraStream(camera.getCamera(), 0);

        Interval interval = new Interval(() -> {
            if (adjust) {
                double posAngle = angleAccum / angleNum;
                posAngle = Math.min(Math.max(posAngle, 0), 270);

                // // Check if angle has changed significantly
                if (Math.abs(posAngle - lastAngle) > ANGLE_DIFFERENCE_THRESHOLD) {
                    // Rumble to indicate significant angle change
                    gamepad1.rumble(100); // Short rumble for 100ms
                }

                lowslide.spinclawSetPositionDeg(posAngle);
                lastAngle = posAngle;
                wasAdjusting = true;
            } else if (wasAdjusting) {
                // When adjustment ends, give feedback
                gamepad1.rumbleBlips(2);
                wasAdjusting = false;
            }
            angleAccum = 0;
            angleNum = 0;
        }, ConfigVariables.General.CAMERA_INTERVAL);
        while (opModeIsActive()) {

            lowslide.spinclawSetPositionDeg(ConfigVariables.General.CLAW_ZERO_DEG);

            double time = System.currentTimeMillis();

            /*
             * player 1
             */
            if (gamepad1.x)
                lowslide.setSlidePos1();
            if (gamepad1.y)
                lowslide.setSlidePos2();

            if (gamepad1.dpad_down)
                lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO);
            if (gamepad1.dpad_right)
                lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO + 45);
            if (gamepad1.dpad_up)
                lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.ZERO + 90);

            if (gamepad1.left_trigger > 0) {
                lowslide.pos_up();
                adjust = false;
                lowslide.spinclawSetPositionDeg(ConfigVariables.LowerSlideVars.SPINCLAW_DEG);
            }
            if (gamepad1.right_trigger > 0) {
                lowslide.pos_grab();
                adjust = false;
            }

            if (gamepad1.left_bumper) {
                if (time - lastTimeGP1LeftBumperCalled > buttonPressIntervalMS) {
                    lowClawIsOpen = !lowClawIsOpen;
                }
                lastTimeGP1LeftBumperCalled = time;
            }
            if (lowClawIsOpen)
                lowslide.openClaw();
            else
                lowslide.closeClaw();

            if (gamepad1.right_bumper) {
                lowslide.pos_hover();
                adjust = true;
            }

            /*
             * player 2
             */
            if (gamepad2.a)
                upslide.pos0();
            if (gamepad2.x)
                upslide.pos1();
            if (gamepad2.y)
                upslide.pos2();
            if (gamepad2.b)
                upslide.pos3();

            if (gamepad2.right_trigger > 0) {
                upslide.transfer();
            }
            if (gamepad2.left_trigger > 0) {
                upslide.front();
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

            if (gamepad2.left_bumper) {
                if (time - lastTimeGP2LeftBumperCalled > buttonPressIntervalMS) {
                    upClawIsOpen = !upClawIsOpen;
                }
                lastTimeGP2LeftBumperCalled = time;
            }
            if (upClawIsOpen)
                upslide.openClaw();
            else
                upslide.closeClaw();

            upslide.updatePID();
            lowslide.updatePID();

            // Update odometry position
            Localizer.positionArc();

            double angle = camera.getAngle(); // -90 ~ 90
            angle = angle + ANGLE_OFFSET; // 0 ~ 180
            angleAccum += angle;
            angleNum += 1;

            // Create dashboard packet
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // Draw robot position from odometry
            field.setStroke("#3F51B5"); // Material Blue
            DashboardUtil.drawRobot(field, "#3F51B5"); // Draw robot using localizer data

            // Visualize limelight detection (camera)
            if (camera.isDetected()) {
                field.setStroke("#4CAF50"); // Material Green for detection
                field.setFill("#4CAF50");
                double radians = Math.toRadians(angle);
                field.strokeLine(0, 0, 20 * Math.cos(radians), 20 * Math.sin(radians));
                field.fillCircle(20 * Math.cos(radians), 20 * Math.sin(radians), 3);
            }

            addTelemetry("angle", angle);

            /*
             * Mecanum drive - player1
             */
            double y = -gamepad1.left_stick_y; // Y stick value is reversed
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

            // Telemetry
            addTelemetry("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            addTelemetry("upslide arm1", upslide.arm1.getPosition());
            addTelemetry("upslide swing", upslide.swing.getPosition());
            addTelemetry("Heading", botHeading);
            addTelemetry("Status", "Running");
            // addTelemetry("claw", upslide.claw.getPosition());
            // addTelemetry("Distance", lowslide.distance);
            // addTelemetry("State", lowslide.slide.getCurrentPosition());
            // addTelemetry("Power", lowslide.PID(lowslide.distance,
            // lowslide.slide.getCurrentPosition()));

            addTelemetry("right trigger", gamepad2.right_trigger);
            addTelemetry("left trigger", gamepad2.left_trigger);

            addTelemetry("right arm", upslide.arm1.getPosition());
            addTelemetry("left arm", upslide.arm2.getPosition());

            telemetry.update();
            ftControlTelemetry.update(telemetry);

            // Additional robot data
            addTelemetry("Detection", camera.isDetected());
            addTelemetry("Upper Slide Position", upslide.arm1.getPosition());
            addTelemetry("Lower Slide Position", lowslide.slide.getCurrentPosition());
            addTelemetry("Robot Heading", botHeading);

            // limelight
            LLResult result = camera.limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                addTelemetry("LL Latency", captureLatency + targetingLatency);
                addTelemetry("Parse Latency", parseLatency);
                addTelemetry("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                addTelemetry("result", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    addTelemetry("tx", result.getTx());
                    addTelemetry("txnc", result.getTxNC());
                    addTelemetry("ty", result.getTy());
                    addTelemetry("tync", result.getTyNC());
                    addTelemetry("Botpose", botpose.toString());
                } else {
                    addTelemetry("Limelight", "No data available");
                }
            }

            LLStatus status = camera.limelight.getStatus();
            addTelemetry("Name", "%s", status.getName());
            addTelemetry("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            addTelemetry("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            dashboard.sendTelemetryPacket(packet);
        }
        interval.cancel();
    }
}
