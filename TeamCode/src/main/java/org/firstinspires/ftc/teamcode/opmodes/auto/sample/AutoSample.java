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

                // Simple pickup and score sequence
                Actions.runBlocking(
                                new SequentialAction(
                                                // score preload
                                                new ParallelAction(
                                                                drive.actionBuilder(START.pose)
                                                                                .strafeToLinearHeading(SCORE.pos,
                                                                                                SCORE.heading)
                                                                                .build(),
                                                                upperSlideCommands.closeClaw(), // double
                                                                                                // makesure/check?
                                                                new SequentialAction(
                                                                                upperSlideCommands.front(),
                                                                                upperSlideCommands.setSlidePos(ConfigVariables.AutoTesting.UPPERSLIDE_POS_3) // pos 3
                                                                                                               // for
                                                                                                               // score
                                                                                                               // height
                                                                )),
                                                // drop to score, upperslide comback in next parallel action
                                                drive.actionBuilder(SCORE.pose).waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S).build(),
                                                new ParallelAction(
                                                                upperSlideCommands.openClaw(),
                                                                lowerSlideCommands.setSlidePos(ConfigVariables.AutoTesting.lowerslideextendlength) // 34cm
                                                ),
                                                upperSlideCommands.scorespec(),

                                                // Drive to pickup while extending arm
                                                new ParallelAction(
                                                                drive.actionBuilder(START.pose)
                                                                                .strafeToLinearHeading(PICKUP1.pos,
                                                                                                PICKUP1.heading)
                                                                                .build(),
                                                                upperSlideCommands.slidePos0()),

                                                // Grab
                                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),
                                                // Drive to score while transferring
                                                new ParallelAction(
                                                                drive.actionBuilder(PICKUP1.pose)
                                                                                .strafeToLinearHeading(SCORE.pos,
                                                                                                SCORE.heading)
                                                                                .build(),
                                                                new SequentialAction(
                                                                                lowerSlideCommands.up(),
                                                                                drive.actionBuilder(SCORE.pose).waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S).build(),
                                                                                upperSlideCommands.transfer(),
                                                                                drive.actionBuilder(SCORE.pose).waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S).build(),
                                                                                lowerSlideCommands.openClaw(),
                                                                                drive.actionBuilder(SCORE.pose).waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S).build(),
                                                                                upperSlideCommands.closeClaw(),
                                                                                drive.actionBuilder(SCORE.pose).waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S).build(),
                                                                                upperSlideCommands.front(),
                                                                                upperSlideCommands.setSlidePos(ConfigVariables.AutoTesting.UPPERSLIDE_POS_3))),
                                                // Drop
                                                drive.actionBuilder(SCORE.pose).waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S).build(),
                                                upperSlideCommands.openClaw()));

                // .strafeToLinearHeading(PICKUP1.pos, PICKUP1.heading)
                // .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                // .strafeToLinearHeading(PICKUP2.pos, PICKUP2.heading)
                // .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                // .strafeToLinearHeading(PICKUP3.pos, PICKUP3.heading)
                // .strafeToLinearHeading(SCORE.pos, SCORE.heading)

                // .strafeToLinearHeading(new Vector2d(38,12), Math.toRadians(180))
                //
                // .strafeToConstantHeading(new Vector2d(23,12))
                //
                // .strafeToConstantHeading(new Vector2d(38,12))
                //
                // .strafeToLinearHeading(SCORE.pos, SCORE.heading);

        }
}
