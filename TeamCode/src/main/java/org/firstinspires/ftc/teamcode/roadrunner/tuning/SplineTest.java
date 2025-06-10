package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous(name = "SplineTest", group = "tuning")
public final class SplineTest extends LinearOpMode {
        // Now using paths from AutoPaths:
        // START = new RobotPosition(40.1, 62, 270);
        // SCORE = new RobotPosition(57, 57, 225);
        // PICKUP1 = new RobotPosition(48.5, 47, -90);
        // PICKUP2 = new RobotPosition(58.5, 47, -90);
        // PICKUP3 = new RobotPosition(55.4, 40.1, -45);
        // PREPLACED = new RobotPosition(47, 46, -90);

        private LowerSlide lowSlide;
        private UpperSlide upSlide;

        private MecanumDrive drive;

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
        }

        @Override
        public void runOpMode() throws InterruptedException {
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Initialize slides
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Build actions using AutoSample paths
                Action moveToScore = drive.actionBuilder(START.pose)
                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                .build();

                Action moveToPickup1 = drive.actionBuilder(SCORE.pose)
                                .strafeToLinearHeading(PICKUP1.pos, PICKUP1.heading)
                                .build();

                Action moveBackToScore = drive.actionBuilder(PICKUP1.pose)
                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                .build();

                Action moveToPickup2 = drive.actionBuilder(SCORE.pose)
                                .strafeToLinearHeading(PICKUP2.pos, PICKUP2.heading)
                                .build();

                Action moveBackToScore2 = drive.actionBuilder(PICKUP2.pose)
                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                .build();

                Action moveToPickup3 = drive.actionBuilder(SCORE.pose)
                                .strafeToLinearHeading(PICKUP3.pos, PICKUP3.heading)
                                .build();

                Action moveBackToScore3 = drive.actionBuilder(PICKUP3.pose)
                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                .build();

                Action moveToStart = drive.actionBuilder(SCORE.pose)
                                .strafeToLinearHeading(START.pos, START.heading)
                                .build();

                waitForStart();

                while (opModeIsActive()) {

                        Actions.runBlocking(moveToScore);

                        waitSeconds(SCORE.pose, 1);

                        Actions.runBlocking(moveToPickup1);

                        waitSeconds(PICKUP1.pose, 1);

                        Actions.runBlocking(moveBackToScore);

                        waitSeconds(SCORE.pose, 1);

                        Actions.runBlocking(moveToPickup2);

                        waitSeconds(PICKUP2.pose, 1);

                        Actions.runBlocking(moveBackToScore2);

                        waitSeconds(SCORE.pose, 1);

                        Actions.runBlocking(moveToPickup3);

                        waitSeconds(PICKUP3.pose, 1);

                        Actions.runBlocking(moveBackToScore3);

                        waitSeconds(SCORE.pose, 1);

                        Actions.runBlocking(moveToStart);

                        waitSeconds(START.pose, 1);
                }


                telemetry.update();
        }
}
