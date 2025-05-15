package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSampleRR extends LinearOpMode {
        private MecanumDrive drive;
        private LowerSlide lowSlide;
        private UpperSlide upSlide;

        private void scorePosition() {
                upSlide.closeClaw();
                upSlide.scorespec();
                upSlide.pos3();
                upSlide.front();
                upSlide.openClaw();

                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                lowSlide.pos_hover();
                lowSlide.setSlidePos2();

                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                upSlide.scorespec();
        }

        private void pickupAndScore(RobotPosition StartPOSE) {
                upSlide.pos0();
                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                new LowerSlideGrabSequenceCommand(lowSlide);
                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                lowSlide.pos_hover();
                lowSlide.setSlidePos0();
                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                upSlide.transfer();
                lowSlide.pos_up();
                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                lowSlide.openClaw();
                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());

                upSlide.closeClaw();
                Actions.runBlocking(
                                drive.actionBuilder(drive.localizer.getPose())
                                                .waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S)
                                                .build());



                // Drive to score position
                Actions.runBlocking(
                        drive.actionBuilder(StartPOSE.pose)
                                .strafeToLinearHeading(StartPOSE.pos, StartPOSE.heading)
                                .build());
                scorePosition();
        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize subsystems
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap);
                upSlide.initialize(hardwareMap);

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Start position
                lowSlide.pos_up();
                upSlide.offwall();
                upSlide.closeClaw();
                Actions.runBlocking(
                                drive.actionBuilder(START.pose)
                                                .waitSeconds(1.0)
                                                .build());

                waitForStart();
                if (isStopRequested())
                        return;

                // Drive to score position
                Actions.runBlocking(
                                drive.actionBuilder(START.pose)
                                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                .build());

                scorePosition();

                // First pickup
                Actions.runBlocking(
                                drive.actionBuilder(SCORE.pose)
                                                .strafeToLinearHeading(PICKUP1.pos, PICKUP1.heading)
                                                .build());
                pickupAndScore(PICKUP1);

                // Second pickup
                Actions.runBlocking(
                                drive.actionBuilder(SCORE.pose)
                                                .strafeToLinearHeading(PICKUP2.pos, PICKUP2.heading)
                                                .build());
                pickupAndScore(PICKUP2);

                // Third pickup
                Actions.runBlocking(
                                drive.actionBuilder(SCORE.pose)
                                                .strafeToLinearHeading(PICKUP3.pos, PICKUP3.heading)
                                                .build());
                pickupAndScore(PICKUP3);

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();

                // Update PID one final time
                lowSlide.updatePID();
        }
}
