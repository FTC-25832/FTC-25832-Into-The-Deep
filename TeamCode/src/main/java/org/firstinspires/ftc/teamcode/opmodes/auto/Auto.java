package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.base.SaveRobotStateCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.opmodes.auto.sample.AutoSample;
import org.firstinspires.ftc.teamcode.opmodes.auto.specimen.AutoSpecimenPush;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;

/**
 * Unified Auto selector that allows choosing between Sample and Specimen
 * autonomous modes
 * This bridges the copied Auto.java functionality to work with your existing
 * codebase
 */
@Autonomous(name = "B. Auto Selector", preselectTeleOp = "A. Teleop")
public class Auto extends LinearOpMode {
        private AutoSelection autoSelection;
        private Limelight limelight;
        private MultipleTelemetry telemetryA;

        // Subsystems for initialization
        private MecanumDrive drive;
        private LowerSlide lowSlide;
        private UpperSlide upSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlideCommands upperSlideCommands;
        private BulkReadManager bulkReadManager;
        private CommandScheduler scheduler;

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize hardware and subsystems
                initializeHardware();

                autoSelection = new AutoSelection(gamepad1);
                telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

                // Initialize limelight
                limelight = new Limelight();
                limelight.initialize(hardwareMap);

                // Initialize subsystems for claw control during init
                scheduler = CommandScheduler.getInstance();

                // Schedule basic initialization commands
                scheduler.schedule(new ActionCommand(upperSlideCommands.closeClaw()));
                scheduler.schedule(new ActionCommand(lowerSlideCommands.up()));
                scheduler.schedule(new LowerSlideUpdatePID(lowSlide));
                scheduler.schedule(new UpperSlideUpdatePID(upSlide));

                while (!isStopRequested() && !opModeIsActive()) {
                        autoSelection.updateTelemetry(telemetryA);
                        telemetryA.addLine("Use D-Pad to select Auto type and Color");
                        telemetryA.addLine("Press Start when ready");
                        telemetryA.update();

                        // Handle claw controls during init
                        TelemetryPacket packet = new TelemetryPacket();

                        if (gamepad1.a) {
                                scheduler.schedule(new ActionCommand(upperSlideCommands.closeClaw()));
                        }
                        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b) {
                                scheduler.schedule(new ActionCommand(upperSlideCommands.openClaw()));
                        }

                        scheduler.run(packet);
                        bulkReadManager.updateBulkRead();

                        sleep(50);
                }

                waitForStart();

                if (isStopRequested())
                        return;

                // Set limelight color based on selection
                String selectedColor = autoSelection.getColor().toString().toLowerCase();
                if (selectedColor.equals("blue")) {
                        limelight.setAcceptedColors(true, false, false); // blue, not red, not yellow
                } else if (selectedColor.equals("red")) {
                        limelight.setAcceptedColors(false, true, false); // not blue, red, not yellow
                } else {
                        limelight.setAcceptedColors(false, false, false); // fallback: none
                }

                // Clean up scheduler before running selected auto
                scheduler.cancelAll();
                scheduler.reset();

                // Run the selected autonomous mode
                switch (autoSelection.getAuto()) {
                        case FULL_SAMPLE:
                                runSampleAuto();
                                break;
                        case SPECIMEN:
                                runSpecimenAuto();
                                break;
                }

                // Save robot state after auto completes
                if (ConfigVariables.General.WITH_STATESAVE) {
                        SaveRobotStateCommand saveCommand = new SaveRobotStateCommand(drive, lowSlide, upSlide);
                        TelemetryPacket packet = new TelemetryPacket();
                        saveCommand.initialize();
                        saveCommand.execute(packet);
                        saveCommand.end(false);
                        FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }

                // Clean up resources
                cleanup();
        }

        private void initializeHardware() {
                // Initialize BulkReadManager for performance optimization
                bulkReadManager = new BulkReadManager(hardwareMap);

                // Initialize subsystems
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Initialize command factories
                lowerSlideCommands = new LowerSlideCommands(lowSlide);
                upperSlideCommands = new UpperSlideCommands(upSlide);

                // Initialize drive with a default starting pose (will be set by specific auto)
                drive = new MecanumDrive(hardwareMap, new com.acmerobotics.roadrunner.Pose2d(0, 0, 0));

                // Register subsystems with scheduler
                scheduler = CommandScheduler.getInstance();
                scheduler.registerSubsystem(upSlide);
                scheduler.registerSubsystem(lowSlide);
        }

        private void runSampleAuto() {
                // Create and run the sample autonomous
                // Note: We need to create a new instance because LinearOpMode can't extend
                // multiple classes
                // So we'll run the AutoSample logic directly

                try {
                        // Create a new AutoSample instance and run its logic
                        AutoSample sampleAuto = new AutoSample();

                        // We can't directly call runOpMode() since that would conflict with this class
                        // Instead, we'll need to extract the core autonomous logic

                        telemetryA.addLine("Running Sample Autonomous...");
                        telemetryA.addData("Color", autoSelection.getColor());
                        telemetryA.update();

                        // For now, indicate that sample auto would run here
                        // In a full implementation, you'd extract the core logic from AutoSample
                        // and run it here, or refactor to use composition instead of inheritance

                        sleep(2000); // Placeholder - replace with actual sample auto logic

                } catch (Exception e) {
                        telemetryA.addLine("Error running Sample Auto: " + e.getMessage());
                        telemetryA.update();
                }
        }

        private void runSpecimenAuto() {
                // Create and run the specimen autonomous
                try {
                        // Similar to sample auto, we'd extract logic from AutoSpecimenPush

                        telemetryA.addLine("Running Specimen Autonomous...");
                        telemetryA.addData("Color", autoSelection.getColor());
                        telemetryA.update();

                        // For now, indicate that specimen auto would run here
                        // In a full implementation, you'd extract the core logic from AutoSpecimenPush
                        // and run it here, or refactor to use composition instead of inheritance

                        sleep(2000); // Placeholder - replace with actual specimen auto logic

                } catch (Exception e) {
                        telemetryA.addLine("Error running Specimen Auto: " + e.getMessage());
                        telemetryA.update();
                }
        }

        private void cleanup() {
                if (autoSelection != null) {
                        autoSelection.interrupt();
                }
                if (scheduler != null) {
                        scheduler.cancelAll();
                        scheduler.reset();
                }
                if (upSlide != null) {
                        upSlide.stop();
                }
                if (lowSlide != null) {
                        lowSlide.stop();
                }
        }
}
