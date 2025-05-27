package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;
import com.example.meepmeeptesting.shared.RobotState;
import com.example.meepmeeptesting.shared.RobotState.Phase;

/**
 * Test OpMode for validating autonomous path planning and state transitions.
 * Tests approach sequences, speed control, and state validation.
 */
@TeleOp(name = "Test: Auto Paths", group = "Test")
public class TestAutoPathsOpMode extends BaseOpMode {
        private MecanumDrive drive;
        private RobotState currentState;
        private boolean isRunningPath = false;
        private boolean lastAState = false;
        private boolean lastBState = false;
        private boolean lastXState = false;
        private boolean lastYState = false;

        @Override
        protected void initialize() {
                drive = new MecanumDrive(hardwareMap, RobotState.START.getPose());
                currentState = RobotState.START;

                displayControls();
        }

        private void displayControls() {
                telemetry.addLine("=== Auto Path Test Controls ===");
                telemetry.addData("A Button", "Run Score Sequence");
                telemetry.addData("B Button", "Run Pickup Sequence");
                telemetry.addData("X Button", "Run Hang Sequence");
                telemetry.addData("Y Button", "Reset to Start");
                telemetry.addLine("\nCurrent State: " + currentState);
                telemetry.addData("Phase", currentState.getPhase());
                telemetry.update();
        }

        @Override
        protected void update() {
                drive.updatePoseEstimate();

                if (!isRunningPath) {
                        handleGamepadInput();
                }

                if (isRunningPath) {
                        // Check if path is complete
                        if (!drive.isBusy()) {
                                isRunningPath = false;
                                telemetry.addData("Path Complete", "Ready for next command");
                        }
                }
        }

        private void handleGamepadInput() {
                // Score sequence
                boolean currentAState = gamepad1.a;
                if (currentAState && !lastAState) {
                        runScoreSequence();
                }
                lastAState = currentAState;

                // Pickup sequence
                boolean currentBState = gamepad1.b;
                if (currentBState && !lastBState) {
                        runPickupSequence();
                }
                lastBState = currentBState;

                // Hang sequence
                boolean currentXState = gamepad1.x;
                if (currentXState && !lastXState) {
                        runHangSequence();
                }
                lastXState = currentXState;

                // Reset position
                boolean currentYState = gamepad1.y;
                if (currentYState && !lastYState) {
                        resetToStart();
                }
                lastYState = currentYState;
        }

        private void runScoreSequence() {
                if (!currentState.canTransitionTo(RobotState.SCORE_POSITION)) {
                        telemetry.addData("Error", "Invalid transition to scoring position");
                        return;
                }

                isRunningPath = true;
                RobotState targetState = RobotState.SCORE_POSITION;

                // Build path with approach point
                drive.followTrajectorySequenceAsync(
                                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                .setReversed(true)
                                                .splineToLinearHeading(
                                                                targetState.getApproachState().getPose(),
                                                                targetState.getApproachState().getHeadingRadians())
                                                .splineToLinearHeading(
                                                                targetState.getPose(),
                                                                targetState.getHeadingRadians())
                                                .build());

                currentState = targetState;
        }

        private void runPickupSequence() {
                if (!currentState.canTransitionTo(RobotState.PICKUP_1)) {
                        telemetry.addData("Error", "Invalid transition to pickup position");
                        return;
                }

                isRunningPath = true;
                RobotState targetState = RobotState.PICKUP_1;

                drive.followTrajectorySequenceAsync(
                                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                .setReversed(true)
                                                .splineToLinearHeading(
                                                                targetState.getApproachState().getPose(),
                                                                targetState.getApproachState().getHeadingRadians())
                                                .splineToLinearHeading(
                                                                targetState.getPose(),
                                                                targetState.getHeadingRadians())
                                                .build());

                currentState = targetState;
        }

        private void runHangSequence() {
                if (!currentState.canTransitionTo(RobotState.HANG_FINAL)) {
                        telemetry.addData("Error", "Invalid transition to hang position");
                        return;
                }

                isRunningPath = true;

                drive.followTrajectorySequenceAsync(
                                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                .splineToLinearHeading(
                                                                RobotState.TANK_APPROACH.getPose(),
                                                                RobotState.TANK_APPROACH.getHeadingRadians())
                                                .splineToLinearHeading(
                                                                RobotState.HANG_START.getPose(),
                                                                RobotState.HANG_START.getHeadingRadians())
                                                .splineToLinearHeading(
                                                                RobotState.HANG_APPROACH.getPose(),
                                                                RobotState.HANG_APPROACH.getHeadingRadians())
                                                .turn(Math.toRadians(-120))
                                                .build());

                currentState = RobotState.HANG_FINAL;
        }

        private void resetToStart() {
                drive.setPoseEstimate(RobotState.START.getPose());
                currentState = RobotState.START;
                isRunningPath = false;
                telemetry.addData("Status", "Reset to start position");
                telemetry.update();
        }

        @Override
        protected void addCustomTelemetry() {
                telemetry.addLine("=== Path Status ===");
                telemetry.addData("Current State", currentState);
                telemetry.addData("Current Phase", currentState.getPhase());
                telemetry.addData("Running Path", isRunningPath);
                telemetry.addData("Speed Multiplier", currentState.getSpeedMultiplier());

                telemetry.addLine("\n=== Robot Pose ===");
                telemetry.addData("X", "%.2f", drive.getPoseEstimate().position.x);
                telemetry.addData("Y", "%.2f", drive.getPoseEstimate().position.y);
                telemetry.addData("Heading", "%.2f°", Math.toDegrees(drive.getPoseEstimate().heading));
        }
}
