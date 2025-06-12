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
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
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

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
        }

        private SequentialAction adjustSequence() {
                return new SequentialAction(
                                new CameraUpdateDetectorResult(camera).toAction(),
                                new DistanceAdjustLUTY(lowSlide, camera::getTy).toAction(),
                                new DistanceAdjustLUTX(drive,
                                                camera::getTx,
                                                camera::getTy, () -> {
                                                }, () -> {
                                                }).toAction(),
                                new WaitCommand(ConfigVariables.Camera.CAMERA_DELAY).toAction());
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

                                new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction(),
                                upperSlideCommands.openExtendoClaw(),
                                new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction(),
                                new SequentialAction(
                                                upperSlideCommands.openClaw(), // drop
                                                // SCORED
                                                new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S)
                                                                .toAction(),
                                                new ParallelAction(
                                                                upperSlideCommands.closeExtendoClaw(),
                                                                upperSlideCommands.scorespec()
                                                // score spec position for upperslides to go down
                                                ),
                                                upperSlideCommands.slidePos0()));

        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                new WaitCommand(ConfigVariables.AutoTesting.Y_PICKUPDELAY).toAction(),
                                // lowerSlideCommands.setSlidePos(lowerslideExtendLength),

                                pickupSequence(),
                                // waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),

                                transferSequence(pickupPos),

                                // Score
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                scoreSequence(pickupPos, lowerslideExtendLength));
        }

        private SequentialAction transferSequence(AutoPaths.RobotPosition pickupPos) {
                return new SequentialAction(
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos2(),
                                // lowerSlideCommands.zero(hardwareMap),

                                // transfer sequence
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.D_SLIDEPOS0AFTERDELAY_S),
                                lowerSlideCommands.up(),
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.E_LOWSLIDEUPAFTERDELAY_S),
                                upperSlideCommands.transfer(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.F_TRANSFERAFTERDELAY_S),
                                upperSlideCommands.closeClaw(),

                                waitSeconds(pickupPos.pose,
                                                ConfigVariables.AutoTesting.G_LOWSLIDETRANSFEROPENCLAWAFTERDELAY_S),
                                lowerSlideCommands.openClaw());
        }

        private SequentialAction pickupSequence() {
                return new SequentialAction(
                                new CameraUpdateDetectorResult(camera).toAction(),
                                new WaitCommand(0.5).toAction(),
                                new DistanceAdjustLUTY(lowSlide, camera::getTy).toAction(),
                                new DistanceAdjustLUTX(drive,
                                                camera::getTx,
                                                camera::getTy, () -> {
                                                }, () -> {
                                                }).toAction(),

                                new WaitCommand(3).toAction(),
                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),

                                new WaitCommand(2).toAction());
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

                // Start position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.scorespec(),
                                                upperSlideCommands.closeClaw()));

                waitForStart();

                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                // Add camera telemetry for debugging
                                                new Action() {
                                                        @Override
                                                        public boolean run(
                                                                        com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                                                                camera.updateTelemetry(packet);
                                                                return true; // Always continue running
                                                        }
                                                },
                                                new SequentialAction(
                                                                upperSlideCommands.scorespec(),

                                                                scoreSequence(START,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),

                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP1,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 45)),
                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP2,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 45)),

                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP3,
                                                                                                0),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 90)),

                                                                // FULL SEND

                                                                drive.actionBuilder(SCORE.pose)
                                                                                .strafeToLinearHeading(
                                                                                                new Vector2d(38, 5),
                                                                                                Math.toRadians(180))
                                                                                .strafeToConstantHeading(
                                                                                                new Vector2d(23, 5))
                                                                                .build(),

                                                                new CameraUpdateDetectorResult(camera).toAction(),
                                                                new WaitCommand(0.5).toAction(),
                                                                new AngleAdjustCommand(lowSlide, camera).toAction(),
                                                                pickupSequence(),
                                                                transferSequence(new RobotPosition(23, 5, 180)),

                                                                drive.actionBuilder(
                                                                                new Pose2d(23, 12, Math.toRadians(180)))
                                                                                .strafeToConstantHeading(
                                                                                                new Vector2d(38, 5))
                                                                                .build(),

                                                                scoreSequence(new RobotPosition(38, 5, 180),
                                                                                ConfigVariables.LowerSlideVars.POS_1_CM)
                                                // .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225))
                                                // .strafeToConstantHeading(SCORE.pos);
                                                // .strafeToLinearHeading(SCORE.pos, SCORE.heading);

                                                )));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
