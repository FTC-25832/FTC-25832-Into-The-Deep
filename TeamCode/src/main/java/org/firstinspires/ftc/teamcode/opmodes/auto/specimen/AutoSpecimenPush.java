package org.firstinspires.ftc.teamcode.opmodes.auto.specimen;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
//import com.bylazar.ftcontrol.panels.plugins.html.primitives.P;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.base.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPathSpecimenTemp.*;

@Autonomous
public final class AutoSpecimenPush extends LinearOpMode {
        private MecanumDrive drive;

        private LowerSlide lowSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlide upSlide;
        private UpperSlideCommands upperSlideCommands;

        private Action waitSeconds(Pose2d pose, double seconds) {
                return drive.actionBuilder(pose)
                                .waitSeconds(seconds)
                                .build();
        }

        private SequentialAction hangSequence(RobotPosition startPOS) {
                return new SequentialAction(
                                waitSeconds(startPOS.pose, 0.5),
                                upperSlideCommands.scorespec(),
                                waitSeconds(startPOS.pose, 0.2),
                                upperSlideCommands.slidePos3());
        }

        private SequentialAction grabFromTeamBox(RobotPosition startPOS) {
                return new SequentialAction(
                                upperSlideCommands.offwall(),
                                upperSlideCommands.openClaw(),
                                waitSeconds(startPOS.pose, 0.2),
                                upperSlideCommands.closeClaw(),
                                upperSlideCommands.front(),
                                waitSeconds(startPOS.pose, 0.1));
        }

        private SequentialAction scoreSequence(RobotPosition startPOS, double lowerslideExtendLength) {
                return new SequentialAction(
                                drive.actionBuilder(startPOS.pose)
                                                .turnTo(135)
                                                .build(),
                                waitSeconds(startPOS.pose, ConfigVariables.AutoTesting.A_DROPDELAY_S),
                                lowerSlideCommands.openClaw());
        }

        private SequentialAction pickupAndScoreSequence(RobotPosition startPOS, RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                return new SequentialAction(
                                // Drive to pickup
                                new ParallelAction(drive.actionBuilder(startPOS.pose)
                                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                                .build(),

                                                lowerSlideCommands.setSlidePos(lowerslideExtendLength)),

                        new SequentialCommandGroup(
                                new ActionCommand(new LowerSlideCommands(lowSlide).openClaw()),
                                new ActionCommand(new LowerSlideCommands(lowSlide).grabPart1()),
                                new ActionCommand(new LowerSlideCommands(lowSlide).grabPart2()),
                                new WaitCommand(ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT/1000.0),
                                new ActionCommand(new LowerSlideCommands(lowSlide).closeClaw()),
                                new WaitCommand(ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT/1000.0),
                                new ActionCommand(new LowerSlideCommands(lowSlide).hover())
                        ).toAction(),
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),
                        // retract, remember to keep pos_hover() when retracting slides
                                // lowerSlideCommands.slidePos0(),

                                // drop to teambox
                                waitSeconds(pickupPos.pose, ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                scoreSequence(pickupPos, lowerslideExtendLength));
        }

        private SequentialAction goHang(RobotPosition goPOS) {
                return new SequentialAction(
                                drive.actionBuilder(new RobotPosition(0, TEST_Y_VALUE4, 270).pose)
                                                .strafeToLinearHeading(new Vector2d(-43, TEST_Y_VALUE),
                                                                Math.toRadians(270))
                                                .build(),
                                grabFromTeamBox(new RobotPosition(-43, TEST_Y_VALUE, 270)),
                                drive.actionBuilder(new RobotPosition(-43, TEST_Y_VALUE, 270).pose)
                                                .strafeToLinearHeading(goPOS.pos, goPOS.heading)
                                                .build(),
                                hangSequence(goPOS));
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
                                                upperSlideCommands.scorespec(),
                                                upperSlideCommands.closeClaw()));

                waitForStart();
                if (isStopRequested())
                        return;

                // Full autonomous sequence
                Actions.runBlocking(
                                new ParallelAction(
                                                new LowerSlideUpdatePID(lowSlide).toAction(),
                                                new UpperSlideUpdatePID(upSlide).toAction(),
                                                new SequentialAction(
                                                                // hang specimen
                                                                drive.actionBuilder(START.pose)
                                                                                .strafeToConstantHeading(new Vector2d(
                                                                                                -8, TEST_Y_VALUE2))
                                                                                .build(),
                                                                hangSequence(new RobotPosition(-8, TEST_Y_VALUE2,
                                                                                Math.toDegrees(START.heading))),
                                                                drive.actionBuilder(new Pose2d(
                                                                                new Vector2d(-8, TEST_Y_VALUE2),
                                                                                START.heading))
                                                                                .strafeTo(new Vector2d(-32,
                                                                                                TEST_Y_VALUE2))
                                                                                .strafeTo(new Vector2d(-51, 12.5))
                                                                                .build(),

                                                                drive.actionBuilder(new Pose2d(-51,12.5, Math.toDegrees(START.heading)))
                                                                        .strafeToConstantHeading(new Vector2d(-8, TEST_Y_VALUE2))
                                                                        .waitSeconds(CLIP_DELAY / 1000.0)
                                                                        .strafeTo(new Vector2d(-32, TEST_Y_VALUE2))
                                                                        .strafeTo(new Vector2d(-48.5, 12.5))
                                                                        .waitSeconds(PICKUP_DELAY / 1000.0)
                                                                        .strafeToLinearHeading(new Vector2d(-48.5, 45), Math.toRadians(-90))
                                                                        .strafeToLinearHeading(new Vector2d(-48.5, 12.5), Math.toRadians(-90))
                                                                        .waitSeconds(PICKUP_DELAY / 1000.0)
                                                                        .strafeToLinearHeading(new Vector2d(-58, 12.5), Math.toRadians(-90))
                                                                        .strafeToLinearHeading(new Vector2d(-58, 45), Math.toRadians(-90))
                                                                        .waitSeconds(PICKUP_DELAY / 1000.0)
                                                                        .strafeToLinearHeading(new Vector2d(-63, 12.5), Math.toRadians(-90))
                                                                        .strafeToLinearHeading(new Vector2d(-63, 45), Math.toRadians(-90))
                                                                        .build(),


                                                                drive.actionBuilder(new Pose2d(-63, 45, -90))
                                                                                .strafeToLinearHeading(new Vector2d(
                                                                                                -41.5, 45),
                                                                                                Math.toRadians(310))
                                                                                .strafeTo(new Vector2d(-41.5,
                                                                                                TEST_Y_VALUE3 - 4))
                                                                                .waitSeconds(0.1)
                                                                                .strafeTo(new Vector2d(-41.5,
                                                                                                TEST_Y_VALUE3))
                                                                                .build(),

                                                                grabFromTeamBox(new RobotPosition(-41.5, TEST_Y_VALUE3,
                                                                                310)),
                                                                drive.actionBuilder(new RobotPosition(-41.5,
                                                                                TEST_Y_VALUE3, 310).pose)
                                                                                .strafeToSplineHeading(new Vector2d(0,
                                                                                                TEST_Y_VALUE4),
                                                                                                Math.toRadians(270))
                                                                                .build(),
                                                                hangSequence(new RobotPosition(0, TEST_Y_VALUE4, 270)),

                                                                goHang(new RobotPosition(-2, TEST_Y_VALUE4, 270)),
                                                                goHang(new RobotPosition(-4, TEST_Y_VALUE4, 270)),
                                                                goHang(new RobotPosition(-6, TEST_Y_VALUE4, 270))

                                                )));

        }
}
