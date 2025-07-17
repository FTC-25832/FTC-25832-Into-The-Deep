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
import org.firstinspires.ftc.teamcode.commands.base.ReadRobotStateCommand;
import org.firstinspires.ftc.teamcode.commands.base.SaveRobotStateCommand;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.hang.HangingCommand;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTX;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTY;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
// import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;

import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
// import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.GamepadController;
import org.firstinspires.ftc.teamcode.utils.GamepadController.ButtonType;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.commands.base.LoopTimeTelemetryCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerUpperTransferSequenceCommand;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@TeleOp(group = "TeleOp")
public class SwerveWithStateSave extends LinearOpMode {

    private MecanumDrive drive;

    private UpperSlide upSlide;
    private LowerSlide lowSlide;
    private Hanging hangingServos;
    // private Limelight camera;

    private UpperSlideCommands upslideActions;
    private LowerSlideCommands lowslideActions;

    private CommandScheduler scheduler;
    private BulkReadManager bulkReadManager;

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
        telemetry = dashboard.getTelemetry();
        scheduler.schedule(new ReadRobotStateCommand(drive, lowSlide, upSlide));

        mecanumDriveCommand = new MecanumDriveCommand(drive, gamepad1);

        scheduler.schedule(mecanumDriveCommand);

        // Schedule loop timing telemetry command
        scheduler.schedule(new LoopTimeTelemetryCommand());

        while (!isStopRequested() && !opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive() && !isStopRequested()) {
            // PERFORMANCE OPTIMIZATION: Update bulk reads once per loop
            bulkReadManager.updateBulkRead();

            TelemetryPacket packet = new TelemetryPacket();
            scheduler.run(packet);

            // DISABLED FOR PERFORMANCE: IMU read adds ~10ms per loop
            // double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // packet.put("heading", heading);

            telemetry.update();

            if (System.currentTimeMillis()
                    - lastDashboardUpdateTime >= ConfigVariables.General.DASHBOARD_UPDATE_INTERVAL_MS) {
                dashboard.sendTelemetryPacket(packet);
                lastDashboardUpdateTime = System.currentTimeMillis();
            }

        }
        scheduler.schedule(new SaveRobotStateCommand(drive, lowSlide, upSlide));
        scheduler.run(new TelemetryPacket());
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

        // Initialize BulkReadManager for performance optimization
        bulkReadManager = new BulkReadManager(hardwareMap);

        upSlide = new UpperSlide();
        lowSlide = new LowerSlide();
        hangingServos = new Hanging();
        // camera = new Limelight();

        scheduler.registerSubsystem(upSlide);
        scheduler.registerSubsystem(lowSlide);
        scheduler.registerSubsystem(hangingServos);

        upSlide.initialize(hardwareMap);
        lowSlide.initialize(hardwareMap);
        hangingServos.initialize(hardwareMap);

        // DISABLED FOR PERFORMANCE: Limelight processing adds ~35ms per loop
        // camera.initialize(hardwareMap);
        // camera.cameraStart();
        // LimeLightImageTools llIt = new LimeLightImageTools(camera.limelight);
        // llIt.setDriverStationStreamSource();
        // llIt.forwardAll();
        // FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(), 10);

        upslideActions = new UpperSlideCommands(upSlide);
        lowslideActions = new LowerSlideCommands(lowSlide);

        upperClaw = new ClawController(upSlide::openClaw, upSlide::closeClaw);
        upperExtendo = new ClawController(upSlide::openExtendoClaw, upSlide::closeExtendoClaw);

        lowerClaw = new ClawController(lowSlide::openClaw, lowSlide::closeClaw);

    }

    private void updatePID() {
        double upslidePower = upSlide.updatePID();
        double lowslidePower = lowSlide.updatePID();
    }
}
