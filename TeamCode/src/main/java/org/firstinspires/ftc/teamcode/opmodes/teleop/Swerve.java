package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

import java.util.Set;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

@TeleOp(group = "TeleOp")
public class Swerve extends LinearOpMode {
    // Constants

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

    // Claw controllers
    private ClawController upperClaw;
    private ClawController lowerClaw;

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
        scheduler.schedule(new ActionCommand(upSlideCommands.front()));
        scheduler.schedule(new ActionCommand(lowSlideCommands.up()));
    }

    private void handleUpperSlideControls() {
        if (gamepad2.right_trigger > 0 && upperClaw.canStartGrabSequence()) {
            Command grabCommand = new UpperSlideGrabSequenceCommand(upSlide);
            upperClaw.startGrabSequence();
            scheduler.schedule(new CommandBase() {
                @Override
                public void initialize() {
                    grabCommand.initialize();
                }

                @Override
                public void execute(TelemetryPacket packet) {
                    grabCommand.execute(packet);
                }

                @Override
                public boolean isFinished() {
                    boolean finished = grabCommand.isFinished();
                    if (finished) {
                        upperClaw.endGrabSequence();
                    }
                    return finished;
                }

                @Override
                public void end(boolean interrupted) {
                    grabCommand.end(interrupted);
                    upperClaw.endGrabSequence();
                }

                @Override
                public Set<SubsystemBase> getRequirements() {
                    return grabCommand.getRequirements();
                }
            });
        }

        if (gamepad2.a)
            scheduler.schedule(new ActionCommand(upSlideCommands.slidePos0()));
        if (gamepad2.x)
            scheduler.schedule(new ActionCommand(upSlideCommands.slidePos1()));
        if (gamepad2.y)
            scheduler.schedule(new ActionCommand(upSlideCommands.slidePos2()));
        if (gamepad2.b)
            scheduler.schedule(new ActionCommand(upSlideCommands.slidePos3()));
        if (gamepad2.dpad_down)
            scheduler.schedule(new ActionCommand(upSlideCommands.transfer()));
        if (gamepad2.dpad_up)
            scheduler.schedule(new ActionCommand(upSlideCommands.front()));
        if (gamepad2.dpad_left)
            scheduler.schedule(new ActionCommand(upSlideCommands.offwall()));
        if (gamepad2.dpad_right)
            scheduler.schedule(new ActionCommand(upSlideCommands.scorespec()));
    }

    private void handleLowerSlideControls() {
        if (gamepad1.right_trigger > 0 && lowerClaw.canStartGrabSequence()) {
            Command grabCommand = new LowerSlideGrabSequenceCommand(lowSlide);
            lowerClaw.startGrabSequence();
            scheduler.schedule(new CommandBase() {
                @Override
                public void initialize() {
                    grabCommand.initialize();
                }

                @Override
                public void execute(TelemetryPacket packet) {
                    grabCommand.execute(packet);
                }

                @Override
                public boolean isFinished() {
                    boolean finished = grabCommand.isFinished();
                    if (finished) {
                        lowerClaw.endGrabSequence();
                    }
                    return finished;
                }

                @Override
                public void end(boolean interrupted) {
                    grabCommand.end(interrupted);
                    lowerClaw.endGrabSequence();
                }

                @Override
                public Set<SubsystemBase> getRequirements() {
                    return grabCommand.getRequirements();
                }
            });
        }

        if (gamepad1.left_trigger > 0) {
            scheduler.schedule(new ActionCommand(lowSlideCommands.up()));
        }

        if (gamepad1.right_bumper) {
            scheduler.schedule(new ActionCommand(lowSlideCommands.hover()));
        }

        if (gamepad1.x)
            scheduler.schedule(new ActionCommand(lowSlideCommands.slidePos1()));
        if (gamepad1.y)
            scheduler.schedule(new ActionCommand(lowSlideCommands.slidePos2()));

        if (gamepad1.dpad_up) {
            Command adjustCommand = new SequentialCommandGroup(
                    new DistanceAdjustCommand(lowSlide, camera),
                    new ActionCommand(lowSlideCommands.hover()),
                    new AngleAdjustCommand(lowSlide, camera)
            );
            scheduler.schedule(adjustCommand);
        }

        if (gamepad1.dpad_down)
            scheduler.schedule(
                    new ActionCommand(lowSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 45)));
        if (gamepad1.dpad_left)
            scheduler.schedule(new ActionCommand(lowSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO)));
        if (gamepad1.dpad_right)
            scheduler.schedule(
                    new ActionCommand(lowSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 90)));
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
        telemetry.addData("lowslide position", lowSlide.pidController.destination);
    }
}
