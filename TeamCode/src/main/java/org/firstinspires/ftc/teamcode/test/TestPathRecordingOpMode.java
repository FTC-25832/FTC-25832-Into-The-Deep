package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.recording.PathRecorder;
import org.firstinspires.ftc.teamcode.utils.visualization.RobotVisualizer;
import com.example.meepmeeptesting.shared.RobotState;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

/**
 * Test OpMode for validating path recording and visualization.
 * Features:
 * - Manual robot movement with gamepad
 * - Path recording start/stop
 * - Real-time visualization
 * - Path comparison with planned trajectories
 */
@TeleOp(name = "Test: Path Recording", group = "Test")
public class TestPathRecordingOpMode extends BaseOpMode {
        private MecanumDrive drive;
        private PathRecorder recorder;
        private RobotVisualizer visualizer;

        private RobotState currentState;
        private String loadedPath = "";
        private List<PathRecorder.PathPoint> plannedPath;
        private boolean isPlayingBack = false;
        private int playbackIndex = 0;

        // Controls state
        private boolean lastAState = false;
        private boolean lastBState = false;
        private boolean lastXState = false;
        private boolean lastYState = false;
        private boolean lastDpadUpState = false;
        private boolean lastDpadDownState = false;

        @Override
        protected void initialize() {
                drive = new MecanumDrive(hardwareMap, RobotState.START.getPose());
                recorder = new PathRecorder(loopTimer);
                visualizer = new RobotVisualizer();
                currentState = RobotState.START;

                displayControls();
        }

        private void displayControls() {
                telemetry.addLine("=== Path Recording Controls ===");
                telemetry.addData("Left Stick", "Drive/Strafe");
                telemetry.addData("Right Stick", "Turn");
                telemetry.addData("A Button", "Start/Stop Recording");
                telemetry.addData("B Button", "Save Recording");
                telemetry.addData("X Button", "Load Last Recording");
                telemetry.addData("Y Button", "Play/Pause Loaded Path");
                telemetry.addData("DPad Up/Down", "Select Loaded Path");
                telemetry.update();
        }

        @Override
        protected void update() {
                // Update drive and get current pose
                drive.updatePoseEstimate();
                Pose2d currentPose = drive.getPoseEstimate();

                handleGamepadInput(currentPose);

                // Update visualization
                visualizer.updateState(
                                currentPose,
                                isPlayingBack ? plannedPath.get(playbackIndex).state : currentState,
                                null,
                                false);

                updateTelemetry(currentPose);
        }

        private void handleGamepadInput(Pose2d currentPose) {
                // Manual robot control
                if (!isPlayingBack) {
                        double drive = -gamepad1.left_stick_y;
                        double strafe = gamepad1.left_stick_x;
                        double turn = gamepad1.right_stick_x;
                        this.drive.setWeightedDrivePower(new Pose2d(drive, strafe, turn));
                }

                // Start/stop recording
                boolean currentAState = gamepad1.a;
                if (currentAState && !lastAState) {
                        toggleRecording();
                }
                lastAState = currentAState;

                // Save recording
                boolean currentBState = gamepad1.b;
                if (currentBState && !lastBState) {
                        recorder.stopRecording();
                        telemetry.addData("Recording", "Saved");
                }
                lastBState = currentBState;

                // Load recording
                boolean currentXState = gamepad1.x;
                if (currentXState && !lastXState) {
                        if (!loadedPath.isEmpty()) {
                                plannedPath = recorder.loadRecording(loadedPath);
                                telemetry.addData("Loaded", loadedPath);
                        }
                }
                lastXState = currentXState;

                // Toggle playback
                boolean currentYState = gamepad1.y;
                if (currentYState && !lastYState) {
                        togglePlayback();
                }
                lastYState = currentYState;

                // Path selection
                boolean currentDpadUpState = gamepad1.dpad_up;
                if (currentDpadUpState && !lastDpadUpState) {
                        // Implement path selection logic
                }
                lastDpadUpState = currentDpadUpState;

                boolean currentDpadDownState = gamepad1.dpad_down;
                if (currentDpadDownState && !lastDpadDownState) {
                        // Implement path selection logic
                }
                lastDpadDownState = currentDpadDownState;

                // Update recording if active
                if (recorder != null) {
                        recorder.recordPoint(currentPose, currentState);
                }

                // Update playback if active
                if (isPlayingBack && plannedPath != null) {
                        updatePlayback();
                }
        }

        private void toggleRecording() {
                if (recorder != null) {
                        if (!isPlayingBack) {
                                recorder.startRecording();
                                telemetry.addData("Recording", "Started");
                        } else {
                                recorder.stopRecording();
                                telemetry.addData("Recording", "Stopped");
                        }
                }
        }

        private void togglePlayback() {
                if (plannedPath != null && !plannedPath.isEmpty()) {
                        isPlayingBack = !isPlayingBack;
                        if (isPlayingBack) {
                                playbackIndex = 0;
                                // Set initial pose
                                drive.setPoseEstimate(plannedPath.get(0).pose);
                        }
                }
        }

        private void updatePlayback() {
                if (playbackIndex < plannedPath.size()) {
                        PathRecorder.PathPoint target = plannedPath.get(playbackIndex);
                        drive.setPoseEstimate(target.pose);
                        currentState = target.state;
                        playbackIndex++;
                } else {
                        isPlayingBack = false;
                        playbackIndex = 0;
                }
        }

        @Override
        protected void addCustomTelemetry() {
                telemetry.addLine("\n=== Path Status ===");
                telemetry.addData("Recording Active", recorder != null);
                telemetry.addData("Playback Active", isPlayingBack);
                telemetry.addData("Current State", currentState);
                telemetry.addData("Loaded Path", loadedPath.isEmpty() ? "None" : loadedPath);

                // Add comparison metrics if available
                if (plannedPath != null && recorder != null) {
                        PathRecorder.PathComparison comparison = recorder
                                        .comparePaths(recorder.loadRecording(loadedPath), plannedPath);
                        telemetry.addLine("\n=== Path Comparison ===");
                        telemetry.addData("Max Position Error", "%.2f inches", comparison.maxPositionError);
                        telemetry.addData("Max Heading Error", "%.2f degrees",
                                        Math.toDegrees(comparison.maxHeadingError));
                        telemetry.addData("Avg Position Error", "%.2f inches", comparison.avgPositionError);
                        telemetry.addData("Avg Heading Error", "%.2f degrees",
                                        Math.toDegrees(comparison.avgHeadingError));
                }
        }
}
