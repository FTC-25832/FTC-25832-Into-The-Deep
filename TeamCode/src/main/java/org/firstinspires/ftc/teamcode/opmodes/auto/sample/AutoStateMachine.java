//package org.firstinspires.ftc.teamcode.opmodes.auto.sample;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
//import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
//import org.firstinspires.ftc.teamcode.commands.vision.VisionAlignAction;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
//import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
//import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
//import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
//
//public class AutoStateMachine {
//        private enum AutoState {
//                INITIALIZE,
//                MOVE_TO_PREPLACE,
//                ALIGN_WITH_VISION,
//                GRAB_SAMPLE,
//                MOVE_TO_SCORE,
//                SCORE_SAMPLE,
//                PARK,
//                DONE
//        }
//
//        private AutoState currentState;
//        private final MecanumDrive drive;
//        private final LowerSlide lowSlide;
//        private final UpperSlide upSlide;
//        private final LowerSlideCommands lowerSlideCommands;
//        private final UpperSlideCommands upperSlideCommands;
//        private final Limelight camera;
//        private final ElapsedTime stateTimer;
//        private final ElapsedTime totalTimer;
//        private static final double STATE_TIMEOUT = 5.0; // 5 second timeout per state
//
//        // Sample positions - these can be modified through dashboard
//        private Pose2d preplacePos;
//        private Pose2d[] samplePositions;
//        private Pose2d scorePos;
//        private Pose2d parkPos;
//        private int currentSampleIndex = 0;
//
//        public AutoStateMachine(MecanumDrive drive, LowerSlide lowSlide, UpperSlide upSlide,
//                        LowerSlideCommands lowerSlideCommands, UpperSlideCommands upperSlideCommands,
//                        Limelight camera) {
//                this.drive = drive;
//                this.lowSlide = lowSlide;
//                this.upSlide = upSlide;
//                this.lowerSlideCommands = lowerSlideCommands;
//                this.upperSlideCommands = upperSlideCommands;
//                this.camera = camera;
//                this.currentState = AutoState.INITIALIZE;
//                this.stateTimer = new ElapsedTime();
//                this.totalTimer = new ElapsedTime();
//
//                // Initialize positions from AutoPaths
//                initializePositions();
//        }
//
//        private void initializePositions() {
//                // These will be loaded from AutoPaths
//                preplacePos = new Pose2d(38.93, 60.23, Math.toRadians(180));
//                samplePositions = new Pose2d[] {
//                                new Pose2d(38.93, 60.23, Math.toRadians(180)),
//                                new Pose2d(38.93, 60.23, Math.toRadians(180)),
//                                new Pose2d(38.93, 60.23, Math.toRadians(180))
//                };
//                scorePos = new Pose2d(38.93, 60.23, Math.toRadians(180));
//                parkPos = new Pose2d(38.93, 60.23, Math.toRadians(180));
//        }
//
//        public Action getNextAction() {
//                if (stateTimer.seconds() > STATE_TIMEOUT) {
//                        telemetry.addData("State Timeout", "Current state: " + currentState);
//                        return handleTimeout();
//                }
//
//                switch (currentState) {
//                        case INITIALIZE:
//                                return initialize();
//                        case MOVE_TO_PREPLACE:
//                                return moveToPosition(preplacePos, AutoState.ALIGN_WITH_VISION);
//                        case ALIGN_WITH_VISION:
//                                return alignWithVision();
//                        case GRAB_SAMPLE:
//                                return grabSample();
//                        case MOVE_TO_SCORE:
//                                return moveToPosition(scorePos, AutoState.SCORE_SAMPLE);
//                        case SCORE_SAMPLE:
//                                return scoreSample();
//                        case PARK:
//                                return moveToPosition(parkPos, AutoState.DONE);
//                        case DONE:
//                                return null;
//                        default:
//                                return null;
//                }
//        }
//
//        private Action handleTimeout() {
//                telemetry.addData("Error", "State timeout occurred");
//                currentState = AutoState.DONE;
//                return null;
//        }
//
//        private Action initialize() {
//                return drive.actionBuilder(drive.pose)
//                                .stopAndAdd(() -> {
//                                        stateTimer.reset();
//                                        currentState = AutoState.MOVE_TO_PREPLACE;
//                                })
//                                .build();
//        }
//
//        private Action moveToPosition(Pose2d targetPos, AutoState nextState) {
//                return drive.actionBuilder(drive.pose)
//                                .strafeToLinearHeading(targetPos.position, targetPos.heading)
//                                .stopAndAdd(() -> {
//                                        stateTimer.reset();
//                                        currentState = nextState;
//                                })
//                                .build();
//        }
//
//        private Action alignWithVision() {
//                VisionAlignAction alignAction = new VisionAlignAction(camera, drive);
//                return alignAction.run(() -> {
//                        stateTimer.reset();
//                        currentState = AutoState.GRAB_SAMPLE;
//                });
//        }
//
//        private Action grabSample() {
//                return drive.actionBuilder(drive.pose)
//                                .stopAndAdd(() -> {
//                                        lowerSlideCommands.grabSequence();
//                                        stateTimer.reset();
//                                        if (currentSampleIndex < samplePositions.length - 1) {
//                                                currentSampleIndex++;
//                                                currentState = AutoState.MOVE_TO_PREPLACE;
//                                        } else {
//                                                currentState = AutoState.MOVE_TO_SCORE;
//                                        }
//                                })
//                                .build();
//        }
//
//        private Action scoreSample() {
//                return drive.actionBuilder(drive.pose)
//                                .stopAndAdd(() -> {
//                                        upperSlideCommands.scoreSequence();
//                                        stateTimer.reset();
//                                        currentState = AutoState.PARK;
//                                })
//                                .build();
//        }
//
//        public void update(TelemetryPacket packet) {
//                packet.put("Current State", currentState);
//                packet.put("Current Sample Index", currentSampleIndex);
//                packet.put("State Timer", stateTimer.seconds());
//                packet.put("Total Time", totalTimer.seconds());
//                packet.put("Drive Pose", drive.pose);
//                packet.put("Lower Slide Position", lowSlide.getCurrentPosition());
//                packet.put("Upper Slide Position", upSlide.getCurrentPosition());
//                packet.put("Camera Target Visible", camera.isTargetVisible());
//                if (camera.isTargetVisible()) {
//                        packet.put("Camera X Offset", camera.getXOffset());
//                        packet.put("Camera Y Offset", camera.getYOffset());
//                }
//        }
//
//        public boolean isDone() {
//                return currentState == AutoState.DONE;
//        }
//
//        // Methods to update positions from dashboard
//        public void updatePreplacePosition(Pose2d newPos) {
//                this.preplacePos = newPos;
//        }
//
//        public void updateSamplePosition(int index, Pose2d newPos) {
//                if (index >= 0 && index < samplePositions.length) {
//                        this.samplePositions[index] = newPos;
//                }
//        }
//
//        public void updateScorePosition(Pose2d newPos) {
//                this.scorePos = newPos;
//        }
//
//        public void updateParkPosition(Pose2d newPos) {
//                this.parkPos = newPos;
//        }
//}