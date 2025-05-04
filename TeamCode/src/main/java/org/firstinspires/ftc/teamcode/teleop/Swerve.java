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

import org.firstinspires.ftc.teamcode.util.ConfigVariables;
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

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

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

            double angle = camera.getAngle(); // -90 ~ 90
            angle = angle + ANGLE_OFFSET; // 0 ~ 180
            angleAccum += angle;
            angleNum += 1;
            // Create dashboard packet
            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            // Draw robot state
            field.setStroke("#3F51B5"); // Material Blue
            field.strokeRect(-18, -18, 36, 36); // Robot base visualization

            // Visualize limelight detection
            if (camera.isDetected()) {
                field.setStroke("#4CAF50"); // Material Green for detection
                field.setFill("#4CAF50");
                double radians = Math.toRadians(angle);
                field.strokeLine(0, 0, 20 * Math.cos(radians), 20 * Math.sin(radians));
                field.fillCircle(20 * Math.cos(radians), 20 * Math.sin(radians), 3);
            }

            // Regular telemetry
            telemetry.addData("angle", angle);

            /*
             * Mecanum drive - player1
             */
            double y = -gamepad1.left_stick_y; // Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // double botHeading = odo.heading();
            doulbe botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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
            telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f), backLeft (%.2f), backRight (%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("upslide arm1", upslide.arm1.getPosition());
            telemetry.addData("upslide swing", upslide.swing.getPosition());
            telemetry.addData("Heading", botHeading);
            telemetry.addData("Status", "Running");
            // telemetry.addData("claw", upslide.claw.getPosition());
            // telemetry.addData("Distance", lowslide.distance);
            // telemetry.addData("State", lowslide.slide.getCurrentPosition());
            // telemetry.addData("Power", lowslide.PID(lowslide.distance,
            // lowslide.slide.getCurrentPosition()));

            telemetry.addData("right", gamepad2.right_trigger);
            telemetry.addData("left", gamepad2.left_trigger);

            telemetry.addData("right", upslide.arm1.getPosition());
            telemetry.addData("left", upslide.arm2.getPosition());

            telemetry.update();

            // Add telemetry data
            packet.put("Angle", angle);
            packet.put("Detection", camera.isDetected());
            packet.put("Upper Slide Position", upslide.arm1.getPosition());
            packet.put("Lower Slide Position", lowslide.slide.getCurrentPosition());
            packet.put("Robot Heading", botHeading);

            LLResult result = camera.limelight.getLatestResult();
            packet.put("result", java.util.Arrays.toString(result.getPythonOutput()));

            dashboard.sendTelemetryPacket(packet);
        }
        interval.cancel();
    }
}
