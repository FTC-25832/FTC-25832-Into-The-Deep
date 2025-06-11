//package org.firstinspires.ftc.teamcode.commands.vision;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
//import org.firstinspires.ftc.teamcode.utils.math.InterpLUT;
//import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
//
//public class VisionAlignAction implements Action {
//        private final Limelight camera;
//        private final MecanumDrive drive;
//        private final InterpLUT lutx = new InterpLUT();
//        private final InterpLUT luty = new InterpLUT();
//        private boolean isAligned = false;
//        private Action moveAction = null;
//        private final ElapsedTime actionTimer = new ElapsedTime();
//        private static final double ACTION_TIMEOUT = 1.5; // 1.5 seconds timeout for movement
//        private static final double ALIGNMENT_THRESHOLD = 0.5; // Threshold for considering alignment complete
//
//        public VisionAlignAction(Limelight camera, MecanumDrive drive) {
//                this.camera = camera;
//                this.drive = drive;
//
//                // Initialize LUTs for X and Y adjustments
//                for (int i = 0; i < ConfigVariables.Camera.X_DISTANCE_MAP_Y.length; i++) {
//                        lutx.add(ConfigVariables.Camera.X_DISTANCE_MAP_X[i],
//                                        ConfigVariables.Camera.X_DISTANCE_MAP_Y[i]);
//                }
//                for (int i = 0; i < ConfigVariables.Camera.Y_DISTANCE_MAP_Y.length; i++) {
//                        luty.add(ConfigVariables.Camera.Y_DISTANCE_MAP_X[i],
//                                        ConfigVariables.Camera.Y_DISTANCE_MAP_Y[i]);
//                }
//                lutx.createLUT();
//                luty.createLUT();
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//                packet.put("vision/aligning", "running");
//
//                // If we have an active movement action, run it
//                if (moveAction != null) {
//                        boolean actionDone = !moveAction.run(packet);
//                        boolean timedOut = actionTimer.seconds() > ACTION_TIMEOUT;
//
//                        if (actionDone || timedOut) {
//                                moveAction = null;
//                                isAligned = true;
//                                packet.put("vision/adjustment", actionDone ? "completed" : "timed out");
//                        } else {
//                                drive.updatePoseEstimate();
//                                return true; // Continue running current action
//                        }
//                }
//
//                // Only detect new adjustments if no active movement
//                if (moveAction == null) {
//                        double dx = camera.getTx();
//                        double dy = camera.getTy();
//
//                        if (dx != 0 && dy != 0) {
//                                // Calculate adjustments using LUTs
//                                double dxcm = lutx.get(dx);
//                                double dycm = luty.get(dy);
//
//                                // Check if we're close enough to target
//                                if (Math.abs(dxcm) < ALIGNMENT_THRESHOLD && Math.abs(dycm) < ALIGNMENT_THRESHOLD) {
//                                        isAligned = true;
//                                        return false;
//                                }
//
//                                // Calculate new pose
//                                Pose2d startpose = drive.localizer.getPose();
//                                double heading = startpose.heading.toDouble();
//
//                                // Convert robot-centric adjustments to field-centric
//                                Vector2d endpose = new Vector2d(
//                                                startpose.position.x - dxcm * Math.sin(heading),
//                                                startpose.position.y - dycm * Math.cos(heading));
//
//                                // Create and start movement action
//                                moveAction = drive.actionBuilder(startpose)
//                                                .strafeToConstantHeading(endpose)
//                                                .build();
//
//                                actionTimer.reset();
//                                drive.updatePoseEstimate();
//                                moveAction.run(packet);
//                        }
//                }
//
//                return !isAligned;
//        }
//}