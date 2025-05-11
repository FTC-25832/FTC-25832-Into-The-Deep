package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

@TeleOp(group = "TeleOp")
public class TestClaw extends LinearOpMode {
    // Localizer odo = new Localizer();

    UpperSlide upslide = new UpperSlide();
    LowerSlide lowslide = new LowerSlide();

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

    private final long lastDashboardUpdateTime = 0;
    private static final long DASHBOARD_UPDATE_INTERVAL_MS = 250; // Update FTCdashboard 4 times per second

    TelemetryPacket packet = new TelemetryPacket();

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

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();


        // odo.initialize(hardwareMap);
        /*
         *
         * TEMP DISABLE, DONT NEED THIS ACCURATE ODO FOR HEADING?
         *
         */

        upslide.initialize(hardwareMap);
        lowslide.initialize(hardwareMap);

        upslide.keepPosExceptArms(0);
        lowslide.keepPosExceptArms(0);

        upslide.front();
        lowslide.pos_up();
        waitForStart();

        while (opModeIsActive()) {

            lowslide.spinclawSetPositionDeg(ConfigVariables.General.CLAW_FORTESTING_DEG);

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

            // Telemetry
//            addTelemetry("upslide arm1", upslide.arm1.getPosition());
//            addTelemetry("upslide swing", upslide.swing.getPosition());
//            addTelemetry("Status", "Running");
//
//            addTelemetry("right arm", upslide.arm1.getPosition());
//            addTelemetry("left arm", upslide.arm2.getPosition());
//
//            // Additional robot data
//            addTelemetry("Upper Slide Position", upslide.arm1.getPosition());
//            addTelemetry("Lower Slide Position", lowslide.slideEncoder.getCurrentPosition());

            telemetry.update();

            dashboard.sendTelemetryPacket(packet);
        }

    }
}
