package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.vision.VisionAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.hang.HangingCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.vision.limelight.Limelight;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {
    // Constants
    private static final long BUTTON_PRESS_INTERVAL_MS = 80;

    // Subsystems
    private Drivetrain drive;
    private UpperSlide upSlide;
    private LowerSlide lowSlide;
    private Hanging hangingServos;
    private Limelight camera;
    private IMU imu;

    // Commands
    private UpperSlideCommands upSlideCommands;
    private LowerSlideCommands lowSlideCommands;

    // Command scheduler
    private CommandScheduler scheduler;

    // Dashboard
    private FtcDashboard dashboard;
    private long lastDashboardUpdateTime = 0;
    private static final long DASHBOARD_UPDATE_INTERVAL_MS = 250;

    // State tracking
    private boolean upClawIsOpen = false;
    private boolean lowClawIsOpen = false;
    private double lastTimeGP1LeftBumperCalled = 0;
    private double lastTimeGP2LeftBumperCalled = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize command scheduler
        scheduler = CommandScheduler.getInstance();

        // Initialize subsystems
        initializeSubsystems();

        // Initialize dashboard
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Set default commands
        drive.setDefaultCommand(new MecanumDriveCommand(drive, gamepad1, imu));

        while (!isStopRequested() && !opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update scheduler with telemetry packet
            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);

            // Handle gamepad inputs
            handleUpperSlideControls();
            handleLowerSlideControls();
            handleHangingControls();
            handleClawControls();

            // Update telemetry
            updateTelemetry();
            telemetry.update();

            // Update dashboard
            if (System.currentTimeMillis() - lastDashboardUpdateTime >= DASHBOARD_UPDATE_INTERVAL_MS) {
                dashboard.sendTelemetryPacket(packet);
                lastDashboardUpdateTime = System.currentTimeMillis();
            }
        }

        // Cleanup
        scheduler.cancelAll();
    }

    private void initializeSubsystems() {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        // Initialize other subsystems
        drive = new Drivetrain();
        upSlide = new UpperSlide();
        lowSlide = new LowerSlide();
        hangingServos = new Hanging();
        camera = new Limelight();

        // Register subsystems with scheduler
        scheduler.registerSubsystem(drive);
        scheduler.registerSubsystem(upSlide);
        scheduler.registerSubsystem(lowSlide);
        scheduler.registerSubsystem(hangingServos);

        // Initialize all subsystems, remember order for same ports so avoid conflicts
        upSlide.initialize(hardwareMap);
        lowSlide.initialize(hardwareMap);
        drive.initialize(hardwareMap);
        hangingServos.initialize(hardwareMap);
        camera.initialize(hardwareMap);
        camera.cameraStart();

        // Initialize command factories
        upSlideCommands = new UpperSlideCommands(upSlide);
        lowSlideCommands = new LowerSlideCommands(lowSlide);

        // Set initial positions
        scheduler.schedule(upSlideCommands.front());
        scheduler.schedule(lowSlideCommands.up());
    }

    private void handleUpperSlideControls() {
        if (gamepad2.a)
            scheduler.schedule(upSlideCommands.pos0());
        if (gamepad2.x)
            scheduler.schedule(upSlideCommands.pos1());
        if (gamepad2.y)
            scheduler.schedule(upSlideCommands.pos2());
        if (gamepad2.b)
            scheduler.schedule(upSlideCommands.pos3());
        if (gamepad2.dpad_down)
            scheduler.schedule(upSlideCommands.transfer());
        if (gamepad2.dpad_up)
            scheduler.schedule(upSlideCommands.front());
        if (gamepad2.dpad_left)
            scheduler.schedule(upSlideCommands.offwall());
        if (gamepad2.dpad_right)
            scheduler.schedule(upSlideCommands.scorespec());
    }

    private void handleLowerSlideControls() {
        if (gamepad1.right_trigger > 0) {
            Command grabCommand = new LowerSlideGrabSequenceCommand(lowSlide);
            scheduler.schedule(grabCommand);
        }

        if (gamepad1.right_bumper) {
            Command adjustCommand = new VisionAdjustCommand(lowSlide, camera);
            scheduler.schedule(adjustCommand);
        }

        if (gamepad1.left_trigger > 0) {
            scheduler.schedule(lowSlideCommands.up());
        }

        if (gamepad1.x)
            scheduler.schedule(lowSlideCommands.slidePos1());
        if (gamepad1.y)
            scheduler.schedule(lowSlideCommands.slidePos2());

        if (gamepad1.dpad_down)
            scheduler.schedule(lowSlideCommands.spinClaw45());
        if (gamepad1.dpad_left)
            scheduler.schedule(lowSlideCommands.spinClaw0());
        if (gamepad1.dpad_right)
            scheduler.schedule(lowSlideCommands.spinClaw90());
    }

    private void handleHangingControls() {
        if (gamepad2.right_trigger > 0)
            scheduler.schedule(new HangingCommand(hangingServos, HangingCommand.Direction.FORWARD));
        if (gamepad2.left_trigger > 0)
            scheduler.schedule(new HangingCommand(hangingServos, HangingCommand.Direction.BACKWARD));
        if (gamepad2.right_bumper)
            scheduler.schedule(new HangingCommand(hangingServos, HangingCommand.Direction.STOP));
    }

    private void handleClawControls() {
        double time = System.currentTimeMillis();

        if (gamepad1.left_bumper && time - lastTimeGP1LeftBumperCalled > BUTTON_PRESS_INTERVAL_MS) {
            lowClawIsOpen = !lowClawIsOpen;
            scheduler.schedule(new ClawToggleCommand(lowSlide, upSlide, false, lowClawIsOpen));
            lastTimeGP1LeftBumperCalled = time;
        }

        if (gamepad2.left_bumper && time - lastTimeGP2LeftBumperCalled > BUTTON_PRESS_INTERVAL_MS) {
            upClawIsOpen = !upClawIsOpen;
            scheduler.schedule(new ClawToggleCommand(lowSlide, upSlide, true, upClawIsOpen));
            lastTimeGP2LeftBumperCalled = time;
        }
    }

    private void updateTelemetry() {
        // Update PID values
        double upslidePower = upSlide.updatePID();
        double lowslidePower = lowSlide.updatePID();

        // Add PID telemetry
        telemetry.addData("upslide power", upslidePower);
        telemetry.addData("upslide position", upSlide.pidfController.destination);
        telemetry.addData("lowslide power", lowslidePower);
        telemetry.addData("lowslide position", lowSlide.pidController.destination);
    }
}
