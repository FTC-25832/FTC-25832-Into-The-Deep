package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSample extends LinearOpMode {

        private SequentialAction scoreSequence(MecanumDrive drive, LowerSlideCommands lowerSlideCommands,
                        UpperSlideCommands upperSlideCommands) {
                return new SequentialAction(
                                drive.actionBuilder(SCORE.pose)
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build(),
                                new ParallelAction(
                                                upperSlideCommands.openClaw(),
                                                lowerSlideCommands.setSlidePos(
                                                                ConfigVariables.AutoTesting.lowerslideextendlength)),
                                upperSlideCommands.scorespec());
        }

        private SequentialAction pickupAndScoreSequence(AutoPaths.RobotPosition pickupPos, MecanumDrive drive,
                        LowerSlideCommands lowerSlideCommands, UpperSlideCommands upperSlideCommands,
                        LowerSlide lowSlide) {
                return new SequentialAction(
                                // Drive to pickup while extending arm
                                new ParallelAction(
                                                drive.actionBuilder(START.pose)
                                                                .strafeToLinearHeading(pickupPos.pos,
                                                                                pickupPos.heading)
                                                                .build(),
                                                upperSlideCommands.slidePos0()),

                                // Grab
                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),
                                // Drive to score while transferring
                                new ParallelAction(
                                                drive.actionBuilder(pickupPos.pose)
                                                                .strafeToLinearHeading(SCORE.pos,
                                                                                SCORE.heading)
                                                                .build(),
                                                new SequentialAction(
                                                                lowerSlideCommands.up(),
                                                                lowerSlideCommands.setSlidePos(0),
                                                                drive.actionBuilder(SCORE.pose)
                                                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                                                .build(),
                                                                upperSlideCommands.transfer(),
                                                                drive.actionBuilder(SCORE.pose)
                                                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                                                .build(),
                                                                lowerSlideCommands.openClaw(),
                                                                drive.actionBuilder(SCORE.pose)
                                                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                                                .build(),
                                                                upperSlideCommands.closeClaw(),
                                                                drive.actionBuilder(SCORE.pose)
                                                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                                                .build(),
                                                                upperSlideCommands.front(),
                                                                upperSlideCommands.setSlidePos(
                                                                                ConfigVariables.AutoTesting.UPPERSLIDE_POS_3))),
                                // Drop
                                drive.actionBuilder(SCORE.pose)
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build(),
                                upperSlideCommands.openClaw());
        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize subsystems
                LowerSlide lowSlide = new LowerSlide();
                UpperSlide upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Initialize command factories
                LowerSlideCommands lowerSlideCommands = new LowerSlideCommands(lowSlide);
                UpperSlideCommands upperSlideCommands = new UpperSlideCommands(upSlide);

                // Initialize drive
                MecanumDrive drive = new MecanumDrive(hardwareMap, START.pose);

                // Start in a safe position
                Actions.runBlocking(
                                new SequentialAction(
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.offwall(),
                                                upperSlideCommands.openClaw(),
                                                upperSlideCommands.closeClaw()));

                waitForStart();
                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new SequentialAction(
                                                // Score preload
                                                new ParallelAction(
                                                                drive.actionBuilder(START.pose)
                                                                                .strafeToLinearHeading(SCORE.pos,
                                                                                                SCORE.heading)
                                                                                .build(),
                                                                upperSlideCommands.closeClaw(),
                                                                new SequentialAction(
                                                                                upperSlideCommands.front(),
                                                                                upperSlideCommands.setSlidePos(
                                                                                                ConfigVariables.AutoTesting.UPPERSLIDE_POS_3))),
                                                scoreSequence(drive, lowerSlideCommands, upperSlideCommands),

                                                // Pickup and score sequences
                                                pickupAndScoreSequence(PICKUP1, drive, lowerSlideCommands,
                                                                upperSlideCommands, lowSlide),
                                                pickupAndScoreSequence(PICKUP2, drive, lowerSlideCommands,
                                                                upperSlideCommands, lowSlide),
                                                pickupAndScoreSequence(PICKUP3, drive, lowerSlideCommands,
                                                                upperSlideCommands, lowSlide)));
        }
}
