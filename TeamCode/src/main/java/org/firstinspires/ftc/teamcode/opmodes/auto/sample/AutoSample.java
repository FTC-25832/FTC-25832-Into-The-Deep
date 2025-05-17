package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSample extends LinearOpMode {
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
                                                upperSlideCommands.slidePos3() // need scorespec or transfer pos to go
                                                                               // up safely
                                ),

                                // front pos for drop
                                upperSlideCommands.front(),
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.A_DROPDELAY_S),

                                new ParallelAction(
                                                upperSlideCommands.openClaw(), // drop
                                                // SCORED

                                                // lowerslide prepare for next cycle
                                                lowerSlideCommands.hover(),
                                                lowerSlideCommands.setSlidePos(lowerslideExtendLength)// EXTEND
                                ),
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.B_DROPPEDAFTERDELAY_S),
                                upperSlideCommands.scorespec()); // score spec position for upperslides to go down
        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                new ParallelAction(
                                                // upperslides go down
                                                upperSlideCommands.slidePos0(),
                                                // Grab
                                                new LowerSlideGrabSequenceCommand(lowSlide).toAction()),
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos0(),
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

                camera = new Limelight();

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
                                                new SequentialAction(
                                                                upperSlideCommands.scorespec(),
                                                                scoreSequence(START,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),
                                                                pickupAndScoreSequence(SCORE, PICKUP1,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                                pickupAndScoreSequence(SCORE, PICKUP2,
                                                                                ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),

                                                                lowerSlideCommands.setSpinClawDeg(
                                                                                ConfigVariables.LowerSlideVars.ZERO
                                                                                                + 90),
                                                                pickupAndScoreSequence(SCORE, PICKUP3, 0),
                                                                upperSlideCommands.setSlidePos(0),
                                                                lowerSlideCommands.up(),
                                                                upperSlideCommands.front(),

                                                                // end pos for teleop
                                                                lowerSlideCommands.slidePos0(),
                                                                upperSlideCommands.slidePos0()
                                                // drive.actionBuilder(SCORE.pose)
                                                // .turnTo(TELEOP_START.heading)
                                                // .build(),
                                                //
                                                // drive.actionBuilder(new Pose2d(
                                                // SCORE.pos,
                                                // TELEOP_START.heading))
                                                // .strafeToLinearHeading(TELEOP_START.pos,
                                                // TELEOP_START.heading)
                                                // .build()
                                                )));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
