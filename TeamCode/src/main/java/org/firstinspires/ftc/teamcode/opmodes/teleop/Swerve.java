package org.firstinspires.ftc.teamcode.opmodes.teleop;

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
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.hang.HangingCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideScoreCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCalculatedX;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCalculatedY;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTThetaR;
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
import org.firstinspires.ftc.teamcode.commands.base.LoopTimeTelemetryCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerUpperTransferSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {

    private MecanumDrive drive;

    private UpperSlide upSlide;
    private LowerSlide lowSlide;
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

    private MecanumDriveCommand mecanumDriveCommand;

    @Override
    public void runOpMode() throws InterruptedException {

        scheduler = CommandScheduler.getInstance();

        initializeSubsystems();

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        mecanumDriveCommand = new MecanumDriveCommand(drive, gamepad1);
        scheduler.schedule(mecanumDriveCommand);
        // Schedule loop timing telemetry command
        scheduler.schedule(new LoopTimeTelemetryCommand());
        scheduler.schedule(new ActionCommand(upslideActions.front()));
        scheduler.schedule(new ActionCommand(lowslideActions.up()));
        setupGamepadControls();
        setContinuousControls();
        setClawControls();

        while (!isStopRequested() && !opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);
//            gamepad1Controller.update();
//            gamepad2Controller.update();
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive() && !isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);

            gamepad1Controller.update();
            gamepad2Controller.update();
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
        scheduler.reset();
    }

    private void initializeSubsystems() {
        drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, 0));

        upSlide = new UpperSlide();
        lowSlide = new LowerSlide();
        camera = new Limelight();

        scheduler.registerSubsystem(upSlide);
        scheduler.registerSubsystem(lowSlide);

        upSlide.initialize(hardwareMap);
        lowSlide.initialize(hardwareMap);
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

        scheduler.schedule(new UpperSlideUpdatePID(upSlide));
        scheduler.schedule(new LowerSlideUpdatePID(lowSlide));
    }

    private void setupGamepadControls() {

        gamepad1Controller.onPressed(ButtonType.X, () -> {
            scheduler.schedule(new CameraUpdateDetectorResult(camera));
            scheduler.schedule(new DistanceAdjustLUTX(drive, camera::getTx, camera::getTy, camera::getPx, camera::getPy, mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
            scheduler.schedule(new DistanceAdjustLUTY(lowSlide, camera::getTy));
            // scheduler.schedule(new DistanceAdjustLUTX(drive, camera::getTx,
            // camera::getTy,camera::getPx, camera::getPy,
            // mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
            // scheduler.schedule(new SequentialCommandGroup(
            // new DistanceAdjustLUTY(lowSlide, camera, gamepad1),
            // new WaitCommand(0.5),
            // new DistanceAdjustLUTX(camera, gamepad1, drive,
            // mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl)
            // ));
        });

        gamepad1Controller.onPressed(ButtonType.DPAD_UP, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
            scheduler.schedule(new AngleAdjustCommand(lowSlide, camera));

        });

        gamepad1Controller.onPressed(ButtonType.RIGHT_BUMPER, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.hover()));
        });

        gamepad1Controller.onPressed(ButtonType.A, () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.slidePos0()));
        });

        gamepad1Controller.onPressed(ButtonType.B, () -> {
            scheduler.schedule(new CameraUpdateDetectorResult(camera));
            scheduler.schedule(new DistanceAdjustLUTX(drive, camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                    mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
            // scheduler.schedule(new DistanceAdjustCalculatedY(lowSlide, camera::getDy));
            // scheduler.schedule(new DistanceAdjustCalculatedX(drive, camera::getDx,
            // camera::getDy,
            // mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
        });

        gamepad1Controller.onPressed(ButtonType.Y, () -> {
            scheduler.schedule(new CameraUpdateDetectorResult(camera));
            scheduler.schedule(new DistanceAdjustLUTThetaR(lowSlide, drive,
                    camera::getTx, camera::getTy, camera::getPx, camera::getPy,
                    mecanumDriveCommand::disableControl, mecanumDriveCommand::enableControl));
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
        // adding tick not pos, direction reversed
        gamepad2Controller.whilePressed(ButtonType.X, (d) -> {
            scheduler.schedule(new ActionCommand(upslideActions.addSlideTick(1)));
        });

        gamepad2Controller.whilePressed(ButtonType.Y, (d) -> {
            scheduler.schedule(new ActionCommand(upslideActions.addSlideTick(-1)));
        });

        gamepad2Controller.onPressed(ButtonType.B, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.slidePos3()));
        });

        gamepad2Controller.onPressed(ButtonType.DPAD_DOWN, () -> {
            scheduler.schedule(new ActionCommand(upslideActions.inter()));
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

        gamepad2Controller.onPressed(ButtonType.RIGHT_STICK_BUTTON, () -> {
            scheduler.schedule(new LowerUpperTransferSequenceCommand(lowslideActions, upslideActions));
        });
        gamepad2Controller.onPressed(ButtonType.LEFT_STICK_BUTTON, () -> {
            scheduler.schedule(new UpperSlideScoreCommand(upslideActions));
        });
    }

    private void setContinuousControls() {
        gamepad1Controller.onPressed(gamepad1Controller.trigger(GamepadController.TriggerType.RIGHT_TRIGGER), () -> {
            scheduler.schedule(new LowerSlideGrabSequenceCommand(lowSlide));
        });
        gamepad1Controller.onPressed(gamepad1Controller.trigger(GamepadController.TriggerType.LEFT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(lowslideActions.up()));
        });

        gamepad2Controller.onPressed(gamepad2Controller.trigger(GamepadController.TriggerType.RIGHT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(upslideActions.transfer()));
        });

        gamepad2Controller.onPressed(gamepad2Controller.trigger(GamepadController.TriggerType.LEFT_TRIGGER), () -> {
            scheduler.schedule(new ActionCommand(upslideActions.front()));
        });
    }

    private void setClawControls() {
        gamepad1Controller.onPressed(gamepad1Controller.button(ButtonType.LEFT_BUMPER), () -> lowerClaw.handleManualControl(System.currentTimeMillis()));
        gamepad2Controller.onPressed(gamepad2Controller.button(ButtonType.LEFT_BUMPER), () -> upperClaw.handleManualControl(System.currentTimeMillis()));
        gamepad2Controller.onPressed(gamepad2Controller.button(ButtonType.RIGHT_BUMPER), () -> upperExtendo.handleManualControl(System.currentTimeMillis()));
    }
}