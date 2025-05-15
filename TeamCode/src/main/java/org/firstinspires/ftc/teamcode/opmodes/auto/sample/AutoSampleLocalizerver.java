package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.roadrunner.Action;
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
public final class AutoSampleLocalizerver extends LinearOpMode {
        private MecanumDrive drive;

        private LowerSlide lowSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlide upSlide;
        private UpperSlideCommands upperSlideCommands;

        private Action waitSeconds(double seconds) {
                return drive.actionBuilder(drive.localizer.getPose())
                                .waitSeconds(seconds)
                                .build();
        }

        private SequentialAction scoreSequence(RobotPosition startPOS) {
                return new SequentialAction(

                                // Drive to score
                                drive.actionBuilder(drive.localizer.getPose())
                                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                                .build(),
                                upperSlideCommands.closeClaw(),
                                // need scorespec position to move slide up
                                upperSlideCommands.scorespec(),
                                upperSlideCommands.slidePos3(),
                                // front pos for drop
                                upperSlideCommands.front(),
                                upperSlideCommands.openClaw(), // drop
                                // SCORED

                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                // lowerslide prepare for next cycle
                                lowerSlideCommands.hover(),
                                lowerSlideCommands.slidePos2(), // EXTEND
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.scorespec()); // score spec position for upperslides to go down
        }

        private SequentialAction pickupAndScoreSequence(AutoPaths.RobotPosition pickupPos) {
                return new SequentialAction(
                                // Drive to pickup
                                drive.actionBuilder(drive.localizer.getPose())
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                // upperslides go down
                                upperSlideCommands.slidePos0(),

                                // Grab
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                new LowerSlideGrabSequenceCommand(lowSlide).toAction(),
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                // retract, remember to keep pos_hover() when retracting slides
                                lowerSlideCommands.slidePos0(),
                                // lowerSlideCommands.zero(hardwareMap),

                                // transfer sequence
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.transfer(),
                                lowerSlideCommands.up(),

                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                lowerSlideCommands.openClaw(),

                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
                                upperSlideCommands.closeClaw(),

                                // Score
                                waitSeconds(ConfigVariables.AutoTesting.DROPDELAY_S),
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
                                new SequentialAction(

                                                scoreSequence(START),

                                                pickupAndScoreSequence(PICKUP1),
                                                pickupAndScoreSequence(PICKUP2),
                                                pickupAndScoreSequence(PICKUP3)

                                ));

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();

                // Update PID one final time
                lowSlide.updatePID();
        }
}
