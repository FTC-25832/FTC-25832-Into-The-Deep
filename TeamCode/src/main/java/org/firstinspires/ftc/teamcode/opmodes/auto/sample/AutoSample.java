package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

@Autonomous
public final class AutoSample extends LinearOpMode {
        private static final class RobotPosition {
                public final Vector2d pos;
                public final double heading;
                public final Pose2d pose;

                public RobotPosition(double x, double y, double headingDegrees) {
                        this.pos = new Vector2d(x, y);
                        this.heading = Math.toRadians(headingDegrees);
                        this.pose = new Pose2d(x, y, this.heading);
                }
        }

        private static final RobotPosition START = new RobotPosition(30.5, 62, 270);
        private static final RobotPosition PREPLACED = new RobotPosition(47, 46, -90);
        private static final RobotPosition PICKUP = new RobotPosition(58, 46, -55);
        private static final RobotPosition SCORE = new RobotPosition(60, 60, 225);

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
                                                lowerSlideCommands.up().toAction(),
                                                upperSlideCommands.front().toAction()));

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
                                                                lowerSlideCommands.slidePos1().toAction()),
                                                // Grab
                                                lowerSlideCommands.grab().toAction(),
                                                // Drive to score while transferring
                                                new ParallelAction(
                                                                drive.actionBuilder(PICKUP.pose)
                                                                                .strafeToLinearHeading(SCORE.pos,
                                                                                                SCORE.heading)
                                                                                .build(),
                                                                new SequentialAction(
                                                                                lowerSlideCommands.up().toAction(),
                                                                                upperSlideCommands.transfer()
                                                                                                .toAction(),
                                                                                lowerSlideCommands.up().toAction(),
                                                                                upperSlideCommands.pos3().toAction())),
                                                // Drop
                                                upperSlideCommands.front().toAction()));
        }
}
