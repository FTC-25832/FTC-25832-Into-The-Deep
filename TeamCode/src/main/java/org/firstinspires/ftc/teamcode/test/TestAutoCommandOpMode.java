package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.commands.auto.AutoCommandSequence;
import org.firstinspires.ftc.teamcode.utils.visualization.RobotVisualizer;
import com.example.meepmeeptesting.shared.RobotState;
import com.acmerobotics.roadrunner.Pose2d;

/**
 * Test OpMode for validating autonomous command sequences.
 * Tests command execution, state transitions, and vision integration.
 */
@TeleOp(name = "Test: Auto Commands", group = "Test")
public class TestAutoCommandOpMode extends BaseOpMode {
        private MecanumDrive drive;
        private VisionStateManager vision;
        private AutoCommandSequence sequence;
        private RobotVisualizer visualizer;

        private boolean lastAState = false;
        private boolean lastBState = false;
        private boolean lastXState = false;
        private boolean lastYState = false;

        private enum TestSequence {
                SIMPLE_DRIVE,
                STATE_TRANSITIONS,
                VISION_ALIGNMENT,
                PARALLEL_ACTIONS
        }

        private TestSequence currentTest = TestSequence.SIMPLE_DRIVE;

        @Override
        protected void initialize() {
                // Initialize subsystems
                drive = new MecanumDrive(hardwareMap, RobotState.START.getPose());
                vision = new VisionStateManager(loopTimer);
                visualizer = new RobotVisualizer();

                displayControls();
        }

        private void displayControls() {
                telemetry.addLine("=== Auto Command Test Controls ===");
                telemetry.addData("A Button", "Start Selected Sequence");
                telemetry.addData("B Button", "Stop Current Sequence");
                telemetry.addData("X Button", "Reset Robot Position");
                telemetry.addData("Y Button", "Cycle Test Sequences");
                telemetry.addData("Current Test", currentTest);
                telemetry.update();
        }

        @Override
        protected void update() {
                // Update subsystems
                drive.updatePoseEstimate();
                vision.update(new ArrayList<>());

                handleGamepadInput();

                // Update active sequence
                if (sequence != null && sequence.isRunning()) {
                        sequence.update();
                }

                // Update visualization
                visualizer.updateState(
                                drive.getPoseEstimate(),
                                currentTest == TestSequence.STATE_TRANSITIONS ? RobotState.SCORE_POSITION : null,
                                vision.getCurrentState(),
                                vision.hasDetection());
        }

        private void handleGamepadInput() {
                // Start sequence with A
                boolean currentAState = gamepad1.a;
                if (currentAState && !lastAState) {
                        startSelectedSequence();
                }
                lastAState = currentAState;

                // Stop sequence with B
                boolean currentBState = gamepad1.b;
                if (currentBState && !lastBState) {
                        if (sequence != null) {
                                sequence.stop();
                        }
                }
                lastBState = currentBState;

                // Reset position with X
                boolean currentXState = gamepad1.x;
                if (currentXState && !lastXState) {
                        drive.setPoseEstimate(RobotState.START.getPose());
                }
                lastXState = currentXState;

                // Cycle tests with Y
                boolean currentYState = gamepad1.y;
                if (currentYState && !lastYState) {
                        currentTest = TestSequence.values()[(currentTest.ordinal() + 1) % TestSequence.values().length];
                        telemetry.addData("Selected Test", currentTest);
                }
                lastYState = currentYState;
        }

        private void startSelectedSequence() {
                sequence = new AutoCommandSequence(drive, vision, loopTimer);

                switch (currentTest) {
                        case SIMPLE_DRIVE:
                                buildSimpleDriveTest();
                                break;
                        case STATE_TRANSITIONS:
                                buildStateTransitionTest();
                                break;
                        case VISION_ALIGNMENT:
                                buildVisionAlignmentTest();
                                break;
                        case PARALLEL_ACTIONS:
                                buildParallelActionsTest();
                                break;
                }

                sequence.start();
        }

        private void buildSimpleDriveTest() {
                // Simple square path
                sequence.addDriveTo(new Pose2d(24, 0, 0))
                                .addDriveTo(new Pose2d(24, 24, Math.PI / 2))
                                .addDriveTo(new Pose2d(0, 24, Math.PI))
                                .addDriveTo(new Pose2d(0, 0, -Math.PI / 2));
        }

        private void buildStateTransitionTest() {
                // Test state transitions with scoring sequence
                sequence.addStateTransition(RobotState.SCORE_APPROACH)
                                .addStateTransition(RobotState.SCORE_POSITION)
                                .addStateTransition(RobotState.SCORE_RETREAT);
        }

        private void buildVisionAlignmentTest() {
                // Test vision-guided alignment
                sequence.addDriveTo(RobotState.PICKUP_1_APPROACH.getPose())
                                .addVisionAlign()
                                .addStateTransition(RobotState.PICKUP_1);
        }

        private void buildParallelActionsTest() {
                // Test parallel command execution
                AutoCommandSequence.Command driveCommand = new AutoCommandSequence.DriveCommand(
                                RobotState.SCORE_POSITION.getPose(), 5.0);
                AutoCommandSequence.Command visionCommand = new AutoCommandSequence.VisionAlignCommand(5.0);

                sequence.addParallel(driveCommand, visionCommand);
        }

        @Override
        protected void addCustomTelemetry() {
                telemetry.addLine("\n=== Sequence Status ===");
                telemetry.addData("Current Test", currentTest);
                telemetry.addData("Sequence Running", sequence != null && sequence.isRunning());
                telemetry.addData("Vision State", vision.getCurrentState());
                telemetry.addData("Has Detection", vision.hasDetection());

                telemetry.addLine("\n=== Robot Pose ===");
                Pose2d pose = drive.getPoseEstimate();
                telemetry.addData("X", "%.2f", pose.position.x);
                telemetry.addData("Y", "%.2f", pose.position.y);
                telemetry.addData("Heading", "%.2f°", Math.toDegrees(pose.heading));
        }
}
