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
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.drive.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.hang.HangingCommand;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import org.firstinspires.ftc.teamcode.utils.gamepad.ButtonEnum;
import org.firstinspires.ftc.teamcode.utils.gamepad.CustomGamepadEx;
import org.firstinspires.ftc.teamcode.commands.base.InstantCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Set;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {
    // Drive system
    private MecanumDrive drive;

    // Subsystems
    private UpperSlide upSlide;
    private LowerSlide lowSlide;
    private Hanging hangingServos;
    private Limelight camera;

    // Commands
    private UpperSlideCommands upslideActions;
    private LowerSlideCommands lowslideActions;

    // Command scheduler
    private CommandScheduler scheduler;

    // Dashboard
    private FtcDashboard dashboard;
    private long lastDashboardUpdateTime = 0;

    // Claw controllers
    private ClawController upperClaw;
    private ClawController lowerClaw;

    private IMU imu;

    // Custom Gamepads
    private CustomGamepadEx driverGamepad;
    private CustomGamepadEx operatorGamepad;

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
        scheduler.schedule(new MecanumDriveCommand(drive, gamepad1, imu));

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

            // Get robot heading from IMU
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            packet.put("heading", heading);

            // Update custom gamepads first
            driverGamepad.update();
            operatorGamepad.update();

            // Update scheduler with telemetry packet
            scheduler.run(packet); // This was already here, ensure custom gamepad updates are before it.

            // Get robot heading from IMU
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            packet.put("heading", heading);

            // Handle manual claw controls (if any remain after full refactor)
            handleClawControls(); // Keep this for now, might be removed if all claw logic moves to commands

            // Update telemetry
            updateTelemetry();
            telemetry.update();

            // Update dashboard
            if (System.currentTimeMillis()
                    - lastDashboardUpdateTime >= ConfigVariables.General.DASHBOARD_UPDATE_INTERVAL_MS) {
                dashboard.sendTelemetryPacket(packet);
                lastDashboardUpdateTime = System.currentTimeMillis();
            }
        }

        // Cleanup when stopping
        cleanup();
    }

    private void cleanup() {
        // Cancel all running commands
        scheduler.cancelAll();

        // Stop all subsystems
        upSlide.stop();
        lowSlide.stop();
        hangingServos.stop();
        // camera.stop();

        // Reset scheduler state
        scheduler.reset();
    }

    private void initializeSubsystems() {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        imu.resetYaw();

        // Initialize drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        // Initialize other subsystems
        upSlide = new UpperSlide();
        lowSlide = new LowerSlide();
        hangingServos = new Hanging();
        camera = new Limelight();

        // Register subsystems with scheduler
        scheduler.registerSubsystem(upSlide);
        scheduler.registerSubsystem(lowSlide);
        scheduler.registerSubsystem(hangingServos);

        // Initialize all subsystems in proper order to avoid port conflicts
        upSlide.initialize(hardwareMap);
        lowSlide.initialize(hardwareMap);
        hangingServos.initialize(hardwareMap);
        camera.initialize(hardwareMap);
        camera.cameraStart();

        // Initialize command factories
        upslideActions = new UpperSlideCommands(upSlide);
        lowslideActions = new LowerSlideCommands(lowSlide);

        // Initialize custom gamepads
        driverGamepad = new CustomGamepadEx(gamepad1, scheduler);
        operatorGamepad = new CustomGamepadEx(gamepad2, scheduler);

        // Configure button bindings
        configureButtonBindings();

        // Initialize claw controllers
        upperClaw = new ClawController(new ClawController.ClawActuator() {
            @Override
            public void openClaw() {
                upSlide.openClaw();
            }

            @Override
            public void closeClaw() {
                upSlide.closeClaw();
            }
        });

        lowerClaw = new ClawController(new ClawController.ClawActuator() {
            @Override
            public void openClaw() {
                lowSlide.openClaw();
            }

            @Override
            public void closeClaw() {
                lowSlide.closeClaw();
            }
        });

        // Set initial positions
        scheduler.schedule(new ActionCommand(upslideActions.front()));
        scheduler.schedule(new ActionCommand(lowslideActions.up()));
    }

    private void configureButtonBindings() {
        // Operator Gamepad (gamepad2)
        operatorGamepad.button(ButtonEnum.RIGHT_TRIGGER)
                .whenPressed(() -> new UpperSlideGrabSequenceCommand(upSlide, upperClaw)); // Pass ClawController

        operatorGamepad.button(ButtonEnum.A).whenPressed(() -> new ActionCommand(upslideActions.slidePos0()));
        operatorGamepad.button(ButtonEnum.X).whenPressed(() -> new ActionCommand(upslideActions.slidePos1()));
        operatorGamepad.button(ButtonEnum.Y).whenPressed(() -> new ActionCommand(upslideActions.slidePos2()));
        operatorGamepad.button(ButtonEnum.B).whenPressed(() -> new ActionCommand(upslideActions.slidePos3()));
        operatorGamepad.button(ButtonEnum.DPAD_DOWN).whenPressed(() -> new ActionCommand(upslideActions.transfer()));
        operatorGamepad.button(ButtonEnum.DPAD_UP).whenPressed(() -> new ActionCommand(upslideActions.front()));
        operatorGamepad.button(ButtonEnum.DPAD_LEFT).whenPressed(() -> new ActionCommand(upslideActions.offwall()));
        operatorGamepad.button(ButtonEnum.DPAD_RIGHT).whenPressed(() -> new ActionCommand(upslideActions.scorespec()));

        // Hanging controls (still on gamepad2 as per original logic)
        // NOTE: The original handleHangingControls used right_trigger and left_trigger.
        // This conflicts with UpperSlideGrabSequenceCommand if it's also on RIGHT_TRIGGER.
        // Assuming a mistake in original prompt and mapping hanging to bumpers or different buttons.
        // For now, I will map to different buttons to avoid direct conflict and allow testing.
        // If they MUST be on triggers, then WHILE_HELD might be needed with careful command construction.
        // Or, the logic in UpperSlideGrabSequenceCommand needs to be WHILE_HELD too.
        // For now, let's use different buttons for hanging to avoid immediate conflict from original code.
        // This part needs clarification if the original intent was trigger-based hanging AND grab.
        // The prompt stated: "operatorGamepad.button(ButtonEnum.RIGHT_TRIGGER_BUTTON).whileHeld(() -> new UpperSlideGrabSequenceCommand(upSlide));"
        // but the original code for handleUpperSlideControls was: "if (gamepad2.right_trigger > 0 && upperClaw.canStartGrabSequence())" which is a WHEN_PRESSED type of behavior.
        // The original handleHangingControls was: "if (gamepad2.right_trigger > 0)"
        // This is a clear conflict.
        // RESOLUTION: I will map hanging to different buttons for now (e.g., START/BACK or unused face buttons if available)
        // and assume UpperSlideGrabSequence is the primary for RIGHT_TRIGGER.
        // For this refactor, I will use START for hanging FORWARD and BACK for hanging BACKWARD.
        // And RIGHT_BUMPER for STOP (as in original)
        operatorGamepad.button(ButtonEnum.START)
                .whileHeld(() -> new HangingCommand(hangingServos, HangingCommand.Direction.FORWARD));
        operatorGamepad.button(ButtonEnum.BACK)
                .whileHeld(() -> new HangingCommand(hangingServos, HangingCommand.Direction.BACKWARD));
        operatorGamepad.button(ButtonEnum.RIGHT_BUMPER) // This was gamepad2.right_bumper in original
                .whenPressed(() -> new HangingCommand(hangingServos, HangingCommand.Direction.STOP));


        // Driver Gamepad (gamepad1)
        driverGamepad.button(ButtonEnum.RIGHT_TRIGGER)
                .whenPressed(() -> new LowerSlideGrabSequenceCommand(lowSlide, lowerClaw)); // Pass ClawController

        driverGamepad.button(ButtonEnum.LEFT_TRIGGER).whenPressed(() -> new ActionCommand(lowslideActions.up()));
        driverGamepad.button(ButtonEnum.RIGHT_BUMPER).whenPressed(() -> new ActionCommand(lowslideActions.hover()));
        driverGamepad.button(ButtonEnum.A).whenPressed(() -> new ActionCommand(lowslideActions.slidePos0()));
        // driverGamepad.button(ButtonEnum.A).whenPressed(() -> new DistanceAdjustCalculated(lowSlide, camera)); // Original commented out

        driverGamepad.button(ButtonEnum.B).whenPressed(() -> new InstantCommand(imu::resetYaw));
        driverGamepad.button(ButtonEnum.X).whenPressed(() -> new ActionCommand(lowslideActions.slidePos1()));
        driverGamepad.button(ButtonEnum.Y).whenPressed(() -> new ActionCommand(lowslideActions.slidePos2()));

        driverGamepad.button(ButtonEnum.DPAD_UP).whenPressed(() -> new SequentialCommandGroup(
                // new DistanceAdjustCommand(lowSlide, camera, gamepad1), // This command would need refactoring for supplier
                new ActionCommand(lowslideActions.hover()),
                new AngleAdjustCommand(lowSlide, camera, gamepad1) // This command also needs a supplier or refactor
        ));

        driverGamepad.button(ButtonEnum.DPAD_DOWN)
                .whenPressed(() -> new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 45)));
        driverGamepad.button(ButtonEnum.DPAD_LEFT)
                .whenPressed(() -> new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO)));
        driverGamepad.button(ButtonEnum.DPAD_RIGHT)
                .whenPressed(() -> new ActionCommand(lowslideActions.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 90)));

        // Manual Claw controls - these were separate.
        // To integrate them into the new system, we'd ideally make them commands.
        // For now, handleClawControls() is kept.
        // If they were to be commands:
        // driverGamepad.button(ButtonEnum.LEFT_BUMPER).whenPressed(() -> new InstantCommand(() -> lowerClaw.handleManualControl(true, System.currentTimeMillis()))).whenReleased(() -> new InstantCommand(() -> lowerClaw.handleManualControl(false, System.currentTimeMillis())));
        // operatorGamepad.button(ButtonEnum.LEFT_BUMPER).whenPressed(() -> new InstantCommand(() -> upperClaw.handleManualControl(true, System.currentTimeMillis()))).whenReleased(() -> new InstantCommand(() -> upperClaw.handleManualControl(false, System.currentTimeMillis())));
        // This is a bit clunky for debounced manual control. Keeping handleClawControls() is simpler for now.
    }

    private void handleClawControls() {
        double time = System.currentTimeMillis();
        // Handle manual claw controls with debouncing
        lowerClaw.handleManualControl(gamepad1.left_bumper, time);
        upperClaw.handleManualControl(gamepad2.left_bumper, time);
    }

    private void updateTelemetry() {
        // Update PID values
        double upslidePower = upSlide.updatePID();
        double lowslidePower = lowSlide.updatePID();

        // Add PID telemetry
        telemetry.addData("upslide power", upslidePower);
        telemetry.addData("upslide position", upSlide.pidfController.destination);
        telemetry.addData("lowslide power", lowslidePower);
        telemetry.addData("lowslide position", lowSlide.pidfController.destination);
    }
}
