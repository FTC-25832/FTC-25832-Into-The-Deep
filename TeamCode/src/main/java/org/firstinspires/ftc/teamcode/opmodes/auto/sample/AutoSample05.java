package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustAutoCommand;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTX;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustLUTY;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSample05 extends LinearOpMode {
        private MecanumDrive drive;

        private LowerSlide lowSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlide upSlide;
        private UpperSlideCommands upperSlideCommands;

        private Limelight camera;
        private CommandScheduler scheduler;

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
        }

        private SequentialAction scoreSequence(RobotPosition startPOS, double lowerslideExtendLength) {
                return new SequentialAction(
                                new ParallelAction(
                                                // Drive to score
                                                drive.actionBuilder(startPOS.pose)
                                                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                                .build(),
                                                upperSlideCommands.closeClaw(),
                                                upperSlideCommands.front(),
                                                upperSlideCommands.slidePos3() // need scorespec or transfer pos to go
                                                                               // up safely
                                ),

                                // front pos for drop

                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.A_DROPDELAY_S),
                                upperSlideCommands.openExtendoClaw(),
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.A_DROPDELAY_S),
                                new SequentialAction(
                                                upperSlideCommands.openClaw(), // drop
                                                // SCORED
                                                waitSeconds(SCORE.pose,
                                                                ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S),
                                                new ParallelAction(
                                                                upperSlideCommands.closeExtendoClaw(),
                                                                upperSlideCommands.scorespec()
                                                // score spec position for upperslides to go down
                                                ),
                                                upperSlideCommands.slidePos0()));

        }

        private SequentialAction pickupAndScoreSequencePreVision(RobotPosition startPOS,
                        AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.Y_PICKUPDELAY),
                                lowerSlideCommands.setSlidePos(lowerslideExtendLength),

                                // Wait for vision processing time
                                waitSeconds(pickupPos.pose, 0.5));
        }

        private SequentialAction pickupAndScoreSequencePostVision(RobotPosition startPOS,
                        AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos0(),
                                // lowerSlideCommands.zero(hardwareMap),

                                // transfer sequence
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.D_SLIDEPOS0AFTERDELAY_S),
                                new ParallelAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.slidePos1()),
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.E_LOWSLIDEUPAFTERDELAY_S),
                                upperSlideCommands.transfer(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.F_TRANSFERAFTERDELAY_S),
                                upperSlideCommands.closeClaw(),

                                waitSeconds(pickupPos.pose,
                                                ConfigVariables.AutoTesting.G_LOWSLIDETRANSFEROPENCLAWAFTERDELAY_S),
                                lowerSlideCommands.openClaw(),

                                // Score
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                scoreSequence(pickupPos, lowerslideExtendLength));
        }

        private void performVisionAdjustment(AutoPaths.RobotPosition pickupPos) {
                // Update camera detection
                camera.updateDetectorResult();

                // Get current tx/ty values
                double tx = camera.getTx();
                double ty = camera.getTy();

                telemetry.addData("Vision TX", tx);
                telemetry.addData("Vision TY", ty);
                telemetry.addData("Camera Available", camera.available);
                telemetry.addData("Result Available", camera.resultAvailable);
                telemetry.update();

                scheduler.schedule(new CameraUpdateDetectorResult(camera));
                scheduler.schedule(new DistanceAdjustLUTY(lowSlide, camera.getTy()));
                scheduler.schedule(new DistanceAdjustLUTX(drive, camera.getTx(), camera.getTy(),
                        ()->{}, ()->{}));

        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize subsystems
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Initialize command factories
                lowerSlideCommands = new LowerSlideCommands(lowSlide);
                upperSlideCommands = new UpperSlideCommands(upSlide);

                // cam
                camera = new Limelight();
                camera.initialize(hardwareMap);
                camera.cameraStart();
                LimeLightImageTools llIt = new LimeLightImageTools(camera.limelight);
                llIt.setDriverStationStreamSource();
                llIt.forwardAll();
                FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(), 10);

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);



                scheduler = CommandScheduler.getInstance();

                // Start position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.scorespec(),
                                                upperSlideCommands.closeClaw()));

                waitForStart();

                if (isStopRequested())
                        return;

                // Start continuous PID updates
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new SequentialAction(
                                                                upperSlideCommands.scorespec(),
                                                                scoreSequence(START,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST))));

                // PICKUP 1 with vision
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new ParallelAction(
                                                                pickupAndScoreSequencePreVision(SCORE, PICKUP1,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                ConfigVariables.LowerSlideVars.ZERO))));

                // Vision processing for PICKUP1
                performVisionAdjustment(PICKUP1);

                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                pickupAndScoreSequencePostVision(SCORE, PICKUP1,
                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND)));

                // PICKUP 2 with vision
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new ParallelAction(
                                                                pickupAndScoreSequencePreVision(SCORE, PICKUP2,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                ConfigVariables.LowerSlideVars.ZERO))));

                // Vision processing for PICKUP2
                performVisionAdjustment(PICKUP2);

                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                pickupAndScoreSequencePostVision(SCORE, PICKUP2,
                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD)));

                // PICKUP 3 with vision
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new ParallelAction(
                                                                pickupAndScoreSequencePreVision(SCORE, PICKUP3, 0),
                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                + 90))));

                // Vision processing for PICKUP3
                performVisionAdjustment(PICKUP3);

                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                pickupAndScoreSequencePostVision(SCORE, PICKUP3, 0)));

                // FULL SEND - Final pickup with vision
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                drive.actionBuilder(SCORE.pose)
                                                                .strafeToLinearHeading(new Vector2d(38, 5),
                                                                                Math.toRadians(180))
                                                                .strafeToConstantHeading(new Vector2d(23, 5))
                                                                .build()));


                // Perform vision adjustments (simplified for final pickup)
                performVisionAdjustment(new AutoPaths.RobotPosition(23, 5, 180));

                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new SequentialAction(
                                                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),
                                                                drive.actionBuilder(
                                                                                new Pose2d(23, 12, Math.toRadians(180)))
                                                                                .strafeToConstantHeading(
                                                                                                new Vector2d(38, 5))
                                                                                .build(),
                                                                scoreSequence(new RobotPosition(38, 5, 180),
                                                                                ConfigVariables.LowerSlideVars.POS_1_CM))));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
