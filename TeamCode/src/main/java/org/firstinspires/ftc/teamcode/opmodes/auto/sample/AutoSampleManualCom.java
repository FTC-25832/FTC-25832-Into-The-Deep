package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSampleManualCom extends LinearOpMode {
        private MecanumDrive drive;

        private LowerSlide lowSlide;
        private UpperSlide upSlide;

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
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                upSlide.closeClaw();
                                                upSlide.scorespec();
                                                upSlide.pos3();
                                                upSlide.front();
                                                upSlide.openClaw();
                                                return true;
                                        }
                                },
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                lowSlide.pos_hover();
                                                lowSlide.setSlidePos2();
                                                return true;
                                        }
                                },
                                waitSeconds(SCORE.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                upSlide.scorespec();
                                                return true;
                                        }
                                });
        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                upSlide.pos0();
                                                return true;
                                        }
                                },
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                new LowerSlideGrabSequenceCommand(lowSlide);
                                                return true;
                                        }
                                },
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                lowSlide.pos_hover();
                                                lowSlide.setSlidePos0();
                                                return true;
                                        }
                                },
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                upSlide.transfer();
                                                lowSlide.pos_up();
                                                return true;
                                        }
                                },
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                lowSlide.openClaw();
                                                return true;
                                        }
                                },
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.DROPDELAY_S),
                                new Action() {
                                        @Override
                                        public boolean run(TelemetryPacket packet) {
                                                upSlide.closeClaw();
                                                return true;
                                        }
                                },

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

                // Initialize drive with starting pose
                drive = new MecanumDrive(hardwareMap, START.pose);

                // Start position
                lowSlide.pos_up();
                upSlide.offwall();
                upSlide.closeClaw();

                waitForStart();
                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new SequentialAction(

                                                scoreSequence(START),

                                                pickupAndScoreSequence(SCORE, PICKUP1),
                                                pickupAndScoreSequence(SCORE, PICKUP2),
                                                pickupAndScoreSequence(SCORE, PICKUP3)

                                ));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();

                // Update PID one final time
                lowSlide.updatePID();
        }
}
