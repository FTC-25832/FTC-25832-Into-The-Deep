package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.vision.AdjustUntilClose;
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.CameraUpdateDetectorResult;
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
import org.firstinspires.ftc.teamcode.commands.drive.SetDriveSpeedCommand;

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

//                                new SetDriveSpeedCommand(75).toAction(),
                                new ParallelAction(
                                                // Drive to score
                                                drive.actionBuilder(startPOS.pose)
                                                                .strafeToSplineHeading(SCORE.pos, SCORE.heading)
                                                                .build(),

                                                upperSlideCommands.closeClaw(),
                                                upperSlideCommands.front(),
                                                upperSlideCommands.slidePos3() // need scorespec or transfer pos to go
                                // up safely

                                ),

                                // front pos for drop

                                // new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction(),
                                // upperSlideCommands.openExtendoClaw(),
                                // new WaitCommand(ConfigVariables.AutoTesting.W_AFTEREXTENDOOPEN_S).toAction(),
                                new SequentialAction(
                                                upperSlideCommands.openClaw(), // drop
                                                // SCORED
                                                new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S)
                                                                .toAction(),
                                                new ParallelAction(
                                                                // upperSlideCommands.closeExtendoClaw(),
                                                                upperSlideCommands.scorespec()
                                                // score spec position for upperslides to go down
                                                ),
                                                upperSlideCommands.slidePos0()));

        }

        private SequentialAction transferAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(

//                                new SetDriveSpeedCommand(75).toAction(),
                                new ParallelAction(
                                                // Drive to score
                                                drive.actionBuilder(startPOS.pose)
                                                                .strafeToSplineHeading(SCORE.pos, SCORE.heading)
                                                                .build(),

                                                new SequentialAction(
                                                                transferSequence(pickupPos),
                                                                // upperSlideCommands.closeClaw(),
                                                                new WaitCommand(ConfigVariables.AutoTesting.X_TRANSFERWHILEDRIVEAFTERTRANSFERDELAY_S)
                                                                                .toAction(),
                                                                upperSlideCommands.front(),
                                                                upperSlideCommands.slidePos3() // need scorespec or
                                                                                               // transfer pos to go
                                                // up safely
                                                )),

                                // front pos for drop

                                new WaitCommand(ConfigVariables.AutoTesting.A_DROPDELAY_S).toAction(),
                                // upperSlideCommands.openExtendoClaw(),
                                // new WaitCommand(ConfigVariables.AutoTesting.W_AFTEREXTENDOOPEN_S).toAction(),
                                new SequentialAction(
                                                upperSlideCommands.openClaw(), // drop
                                                // SCORED
                                                new WaitCommand(ConfigVariables.AutoTesting.B_AFTERSCOREDELAY_S)
                                                                .toAction(),
                                                new ParallelAction(
                                                                // upperSlideCommands.closeExtendoClaw(),
                                                                upperSlideCommands.scorespec()
                                                // score spec position for upperslides to go down
                                                ),
                                                upperSlideCommands.slidePos0()));

        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength, boolean adjustMultipleTimes) {
                return new SequentialAction(
                                // Drive to pickup
//                                new SetDriveSpeedCommand(40).toAction(),

                                new ParallelAction(
                                                drive.actionBuilder(startPOS.pose)
                                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                                .build(),
                                                lowerSlideCommands.setSlidePos(lowerslideExtendLength)),

                                new WaitCommand(ConfigVariables.AutoTesting.Y_PICKUPDELAY).toAction(),
                                adjustMultipleTimes ? adjustMultipleSequence()
                                                : new SequentialAction(
                                                                adjustSequence(),
                                                                new WaitCommand(0.4).toAction()),

                                pickupSequence(),
                                // waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),

                                // transferSequence(pickupPos),

                                // Score
                                // waitSeconds(pickupPos.pose,
                                // ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                transferAndScoreSequence(pickupPos, pickupPos, lowerslideExtendLength));
        }

        private SequentialAction transferSequence(AutoPaths.RobotPosition pickupPos) {
                return new SequentialAction(
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos2(),

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

        private Action pickupSequence() {
                return new SequentialCommandGroup(
                                new ActionCommand(new LowerSlideCommands(lowSlide).openClaw()),
                                new ActionCommand(new LowerSlideCommands(lowSlide).grabPart1()),
                                new ActionCommand(new LowerSlideCommands(lowSlide).grabPart2()),
                                new WaitCommand(ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT / 1000.0),
                                new ActionCommand(new LowerSlideCommands(lowSlide).closeClaw()),
                                new WaitCommand(ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT / 1000.0),
                                new ActionCommand(new LowerSlideCommands(lowSlide).hover())).toAction();
        }

        private Action adjustSequence() {
                return new SequentialAction(
                                new CameraUpdateDetectorResult(camera).toAction(),
                                new DistanceAdjustLUTX(drive, camera::getTx, camera::getTy, () -> {
                                }, () -> {
                                }).toAction(),
                                new DistanceAdjustLUTY(lowSlide, camera::getTy).toAction());
        }

        private Action adjustMultipleSequence() {
                return new AdjustUntilClose(drive, lowSlide, camera::getTx, camera::getTy, () -> {
                }, () -> {
                },
                                camera::updateDetectorResult)
                                .toAction();
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
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST,
                                                                                                false),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 45)),
                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP2,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND,
                                                                                                false),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 45)),

                                                                new ParallelAction(
                                                                                pickupAndScoreSequence(SCORE, PICKUP3,
                                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD,
                                                                                                true),
                                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                                + 90)),

                                                                // FULL SEND

                                                                drive.actionBuilder(SCORE.pose)
                                                                                .strafeToSplineHeading(
                                                                                                new Vector2d(24, 3),
                                                                                                Math.toRadians(180))
                                                                                .build(),
                                                                new WaitCommand(ConfigVariables.AutoTesting.I_SUBDELAY_S)
                                                                                .toAction(),
                                                                new ParallelAction(
                                                                                adjustMultipleSequence(),
                                                                                new AngleAdjustCommand(lowSlide, camera)
                                                                                                .toAction()),
                                                                // adjustMultipleSequence(),
                                                                // new WaitCommand(0.4).toAction(),
                                                                // new AngleAdjustCommand(lowSlide, camera).toAction(),
                                                                new WaitCommand(ConfigVariables.AutoTesting.J_AFTERSUBDELAY_S)
                                                                                .toAction(),
                                                                pickupSequence(),

                                                                transferAndScoreSequence(new RobotPosition(24, 3, 180),
                                                                                new RobotPosition(23, 12, 180),
                                                                                ConfigVariables.LowerSlideVars.POS_1_CM)
                                                // .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(225))
                                                // .strafeToConstantHeading(SCORE.pos);
                                                // .strafeToLinearHeading(SCORE.pos, SCORE.heading);

                                                )));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
