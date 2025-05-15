package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
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

        private Action waitSeconds(double seconds) {
                return drive.actionBuilder(START.pose)
                                .waitSeconds(seconds)
                                .build();
        }

        private SequentialAction scoreSequence(RobotPosition startPOS) {
                return new SequentialAction(
                                new ParallelAction(
                                                drive.actionBuilder(startPOS.pose)
                                                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                                .build(),
                                                upperSlideCommands.closeClaw(),
                                                upperSlideCommands.scorespec(),
                                                new SequentialAction(upperSlideCommands.slidePos3())),
                                upperSlideCommands.front(),

                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),

                                new ParallelAction(
                                                new SequentialAction(
                                                                upperSlideCommands.openClaw()),
                                                lowerSlideCommands.setSlidePos(
                                                                ConfigVariables.AutoTesting.lowerslideextendlength),
                                                lowerSlideCommands.hover()),
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.scorespec()
                                );
        }

        private SequentialAction pickupAndScoreSequence(AutoPaths.RobotPosition pickupPos) {
                return new SequentialAction(
                                // Drive to pickup while extending arm
                                new ParallelAction(
                                                drive.actionBuilder(START.pose)
                                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                                .build(),
                                                upperSlideCommands.slidePos0(),
                                                lowerSlideCommands.hover()),

                                // Grab
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),

                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),

                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),

                                // Drive to score while transferring
                                new SequentialAction(
                                                new ParallelAction(
                                                                lowerSlideCommands.up(),
                                                                lowerSlideCommands.slidePos0()),

                                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),

                                                upperSlideCommands.transfer(),

                                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),

                                                lowerSlideCommands.openClaw(),

                                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),

                                                upperSlideCommands.closeClaw()),

                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                // Score
                                scoreSequence(pickupPos),
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                );
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
                                new SequentialAction(
                                                // Score preload
                                                scoreSequence(START),

                                                // Pickup and score sequences
                                                pickupAndScoreSequence(PICKUP1),

                                                pickupAndScoreSequence(PICKUP2),


                                                pickupAndScoreSequence(PICKUP3),


                                                //drive to tank
                                                drive.actionBuilder(SCORE.pose)
                                                        .strafeToLinearHeading(new Vector2d(38,12), Math.toRadians(180))
                                                        .strafeToConstantHeading(new Vector2d(23,12))
                                                        .build(),

                                                //fullsend



                                                drive.actionBuilder(new Pose2d(23,12, Math.toRadians(180)))
                                                        .strafeToConstantHeading(new Vector2d(38,12))
                                                        .build()

                                ));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
