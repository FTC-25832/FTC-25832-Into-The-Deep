package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
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
import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
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

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
        }

        private SequentialAction scoreSequence(RobotPosition startPOS) {
                return new SequentialAction(

                                // Drive to score
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                .build(),
                                upperSlideCommands.closeClaw(),
                                // need scorespec position to move slide up
                                upperSlideCommands.scorespec(),
                                upperSlideCommands.slidePos3(),
                                // front pos for drop
                                upperSlideCommands.front(),
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.openClaw(), // drop
                                // SCORED

                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                // lowerslide prepare for next cycle
                                lowerSlideCommands.hover(),

                                lowerSlideCommands.slidePos2(), // EXTEND
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.scorespec()); // score spec position for upperslides to go down
        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                // upperslides go down
                                upperSlideCommands.slidePos0(),

                                // Grab
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos0(),
                                // lowerSlideCommands.zero(hardwareMap),

                                // transfer sequence
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                lowerSlideCommands.up(),
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.transfer(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                lowerSlideCommands.openClaw(),

                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.closeClaw(),

                                // Score
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                scoreSequence(pickupPos));
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

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Start position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.offwall(),
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
                                                                scoreSequence(START),
                                                                pickupAndScoreSequence(SCORE, PICKUP1),
                                                                pickupAndScoreSequence(SCORE, PICKUP2),
                                                                pickupAndScoreSequence(SCORE, PICKUP3))));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
