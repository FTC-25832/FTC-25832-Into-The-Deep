package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

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

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.Y_PICKUPDELAY),
                                lowerSlideCommands.setSlidePos(lowerslideExtendLength),
                                // new RaceAction(
                                // new SequentialAction(
                                // new CameraUpdateDetectorResult(camera).toAction(),
                                // new DistanceAdjustLUTY(lowSlide, camera.getTy())
                                // .toAction())),
                                new SequentialAction(
                                                new CameraUpdateDetectorResult(camera).toAction(),
                                                new DistanceAdjustLUTY(lowSlide, camera.getTy())
                                                                .toAction()),

                                new ParallelAction(
                                                new SequentialAction(
                                                                new CameraUpdateDetectorResult(camera).toAction(),
                                                                new AngleAdjustAutoCommand(lowSlide, camera)
                                                                                .toAction()),

                                                // // Grab
                                                // new RaceAction(
                                                // new SequentialAction(
                                                // new CameraUpdateDetectorResult(camera)
                                                // .toAction(),
                                                // new DistanceAdjustLUTX(drive,
                                                // camera.getTx(),
                                                // camera.getTy(), () -> {
                                                // }, () -> {
                                                // }).toAction()),
                                                // new LowerSlideGrabSequenceCommand(
                                                // lowSlide).toAction())
                                                new SequentialAction(
                                                                new CameraUpdateDetectorResult(camera)
                                                                                .toAction(),
                                                                new DistanceAdjustLUTX(drive,
                                                                                camera.getTx(),
                                                                                camera.getTy(), () -> {
                                                                                }, () -> {
                                                                                }).toAction(),
                                                                new LowerSlideGrabSequenceCommand(
                                                                                lowSlide).toAction())),

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

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Start position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.scorespec(),
                                                upperSlideCommands.closeClaw()));

                waitForStart();
                camera.cameraStart();
                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new SequentialAction(
                                                                upperSlideCommands.scorespec(),

                                                                scoreSequence(START,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),

                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP1,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO)),
                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP2,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO)),

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
                                                                new ParallelAction(
                                                                                new DistanceAdjustLUTY(lowSlide,
                                                                                                camera.getTy())
                                                                                                .toAction(),
                                                                                new DistanceAdjustLUTX(drive,
                                                                                                camera.getTx(),
                                                                                                camera.getTy(), () -> {
                                                                                                }, () -> {
                                                                                                }).toAction(),
                                                                                new AngleAdjustAutoCommand(lowSlide,
                                                                                                camera).toAction()),
                                                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),

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
