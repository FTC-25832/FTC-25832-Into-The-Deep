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

                waitForStart();

                while (opModeIsActive()) {
                        Actions.runBlocking(
                                        drive.actionBuilder(START.pose)
                                                        .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(PICKUP1.pos, PICKUP1.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(PICKUP2.pos, PICKUP2.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(PICKUP3.pos, PICKUP3.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                        .waitSeconds(1)
                                                        .strafeToLinearHeading(START.pos, START.heading)
                                                        .waitSeconds(1)
                                                        .build());
                }
        }
}
