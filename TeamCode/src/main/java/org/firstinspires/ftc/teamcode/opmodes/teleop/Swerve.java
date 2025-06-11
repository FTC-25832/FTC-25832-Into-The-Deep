package org.firstinspires.ftc.teamcode.opmodes.teleop;

import androidx.appcompat.app.WindowDecorActionBar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.hang.HangingCommand;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTX;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTY;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;

import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.GamepadController;
import org.firstinspires.ftc.teamcode.utils.GamepadController.ButtonType;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.timing.Timeout;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {

    private MecanumDrive drive;

    private UpperSlide upSlide;
    private LowerSlide lowSlide;
    private Hanging hangingServos;
    private Limelight camera;

    private UpperSlideCommands upslideActions;
    private LowerSlideCommands lowslideActions;

    private CommandScheduler scheduler;

    private FtcDashboard dashboard;
    private long lastDashboardUpdateTime = 0;

    private GamepadController gamepad1Controller;
    private GamepadController gamepad2Controller;

    private ClawController upperClaw;
    private ClawController upperExtendo;
    private ClawController lowerClaw;

    private IMU imu;
    private MecanumDriveCommand mecanumDriveCommand;

    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = CommandScheduler.getInstance();

        initializeSubsystems();

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        mecanumDriveCommand = new MecanumDriveCommand(drive, gamepad1);

        scheduler.schedule(mecanumDriveCommand);

        setupGamepadControls();

        while (!isStopRequested() && !opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);
            gamepad1Controller.update();
            gamepad2Controller.update();
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive() && !isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            packet.put("heading", heading);

            handleContinuousControls();

            gamepad1Controller.update();
            gamepad2Controller.update();

            handleClawControls();

            updatePID();
            telemetry.update();

            if (System.currentTimeMillis()
                    - lastDashboardUpdateTime >= ConfigVariables.General.DASHBOARD_UPDATE_INTERVAL_MS) {
                dashboard.sendTelemetryPacket(packet);
                lastDashboardUpdateTime = System.currentTimeMillis();
            }
        }

        cleanup();
    }

    private void cleanup() {
        scheduler.cancelAll();
        upSlide.stop();
        lowSlide.stop();
        hangingServos.stop();
        scheduler.reset();
    }

    private void initializeSubsystems() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        upSlide = new UpperSlide();
        lowSlide = new LowerSlide();
        hangingServos = new Hanging();
        camera = new Limelight();

        scheduler.registerSubsystem(upSlide);
        scheduler.registerSubsystem(lowSlide);
        scheduler.registerSubsystem(hangingServos);

        upSlide.initialize(hardwareMap);
        lowSlide.initialize(hardwareMap);
        hangingServos.initialize(hardwareMap);
        camera.initialize(hardwareMap);
        camera.cameraStart();
        LimeLightImageTools llIt = new LimeLightImageTools(camera.limelight);
        llIt.setDriverStationStreamSource();
        llIt.forwardAll();
        FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(), 10);

        upslideActions = new UpperSlideCommands(upSlide);
        lowslideActions = new LowerSlideCommands(lowSlide);

        upperClaw = new ClawController(upSlide::openClaw, upSlide::closeClaw);
        upperExtendo = new ClawController(upSlide::openExtendoClaw, upSlide::closeExtendoClaw);

        lowerClaw = new ClawController(lowSlide::openClaw, lowSlide::closeClaw);

        gamepad1Controller = new GamepadController(gamepad1);
        gamepad2Controller = new GamepadController(gamepad2);

        scheduler.schedule(new ActionCommand(upslideActions.front()));
        scheduler.schedule(new ActionCommand(lowslideActions.up()));
    }

    private void setupGamepadControls() {

        gamepad1Controller.onPressed(ButtonType.X, () -> {
            scheduler.schedule(new DistanceAdjustLUTY(lowSlide, gamepad1, camera));
            new Timeout(() -> {
                scheduler.schedule(new DistanceAdjustLUTX(gamepad1, drive, camera, mecanumDriveCommand::disableControl,
                        mecanumDriveCommand::enableControl));
            }, 600);
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_UP, () -> {
            Command adjustCommand = new SequentialCommandGroup(
                    new ActionCommand(lowslideActions.hover()),
                    new AngleAdjustCommand(lowSlide, camera, gamepad1));
            scheduler.schedule(adjustCommand);
        });

        gamepad1Controller.onPressed(ButtonType.RIGHT_BUMPER, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
        });

        gamepad1Controller.onPressed(ButtonType.A, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.slidePos0()));
        });

        gamepad1Controller.onPressed(ButtonType.B, () -> {
            imu.resetYaw();
        });

        gamepad1Controller.onPressed(ButtonType.Y, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.slidePos2()));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_DOWN, () -> {
            scheduler.schedule(
                    new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 45)));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_LEFT, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO)));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_RIGHT, () -> {
            scheduler.schedule(
                    new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 90)));
        });

        gamepad2Controller.onPressed(ButtonType.A, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.slidePos0()));
        });

        gamepad2Controller.onPressed(ButtonType.X, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.slidePos1()));
        });

        gamepad2Controller.onPressed(ButtonType.Y, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.slidePos2()));
        });

        gamepad2Controller.onPressed(ButtonType.B, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.slidePos3()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_DOWN, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.transfer()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_UP, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.front()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_LEFT, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.offwall()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_RIGHT, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.scorespec()));
        });

    }

    private void handleContinuousControls() {
        // Gamepad 1 trigger controls
        if (gamepad1.right_trigger > 0 && lowerClaw.canStartGrabSequence()) {
            Command grabCommand = new LowerSlideGrabSequenceCommand(lowSlide, lowerClaw);
            lowerClaw.startGrabSequence();
            scheduler.schedule(grabCommand);
        }

        if (gamepad1.left_trigger > 0) {
            scheduler.schedule(new ActionCommand(lowslideActions.up()));
        }

        // Gamepad 2 trigger controls
        if (gamepad2.right_trigger > 0 && upperClaw.canStartGrabSequence()) {
            Command grabCommand = new UpperSlideGrabSequenceCommand(upSlide, upperClaw);
            upperClaw.startGrabSequence();
            scheduler.schedule(grabCommand);
        }

        if (gamepad2.right_trigger > 0) {
            scheduler.schedule(new HangingCommand(hangingServos, HangingCommand.Direction.FORWARD));
        }

        if (gamepad2.left_trigger > 0) {
            scheduler.schedule(new HangingCommand(hangingServos, HangingCommand.Direction.BACKWARD));
        }
    }

    private void handleClawControls() {
        double time = System.currentTimeMillis();

        lowerClaw.handleManualControl(gamepad1.left_bumper, time);
        upperClaw.handleManualControl(gamepad2.left_bumper, time);
        upperExtendo.handleManualControl(gamepad2.right_bumper, time);
    }

    private void updatePID() {
        double upslidePower = upSlide.updatePID();
        double lowslidePower = lowSlide.updatePID();
    }
}