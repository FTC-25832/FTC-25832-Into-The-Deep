package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
                                                upperSlideCommands.front()));

                waitForStart();
                if (isStopRequested())
                        return;

                // Simple pickup and score sequence
                Actions.runBlocking(
                                new SequentialAction(
                                                // Drive to pickup while extending arm
                                                new ParallelAction(
                                                                drive.actionBuilder(START.pose)
                                                                                .strafeToLinearHeading(PICKUP.pos,
                                                                                                PICKUP.heading)
                                                                                .build(),
                                                                lowerSlideCommands.slidePos1()),
                                                // Grab
                                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),
                                                // Drive to score while transferring
                                                new ParallelAction(
                                                                drive.actionBuilder(PICKUP.pose)
                                                                                .strafeToLinearHeading(SCORE.pos,
                                                                                                SCORE.heading)
                                                                                .build(),
                                                                new SequentialAction(
                                                                                lowerSlideCommands.up(),
                                                                                upperSlideCommands.transfer(),
                                                                                lowerSlideCommands.up(),
                                                                                upperSlideCommands.slidePos3())),
                                                // Drop
                                                upperSlideCommands.front(),
                                                upperSlideCommands.openClaw()));
        }
}
