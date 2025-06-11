package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous
public final class AutoSample extends LinearOpMode {
        private MecanumDrive drive;
        private LowerSlide lowSlide;
        private UpperSlide upSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlideCommands upperSlideCommands;
        private Limelight camera;
        private AutoStateMachine stateMachine;

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
                drive = new MecanumDrive(hardwareMap, new Pose2d(38.93, 60.23, Math.toRadians(180)));

                // Initialize camera
                camera = new Limelight();
                camera.initialize(hardwareMap);
                camera.cameraStart();

                // Initialize state machine
                stateMachine = new AutoStateMachine(drive, lowSlide, upSlide, lowerSlideCommands, upperSlideCommands,
                                camera);

                waitForStart();
                if (isStopRequested())
                        return;

                // Main autonomous loop
                while (opModeIsActive() && !isStopRequested()) {
                        TelemetryPacket packet = new TelemetryPacket();

                        // Get and run next action from state machine
                        Action nextAction = stateMachine.getNextAction();
                        if (nextAction != null) {
                                Actions.runBlocking(nextAction);
                        }

                        // Update telemetry
                        stateMachine.update(packet);
                        telemetry.update();

                        if (stateMachine.isDone()) {
                                break;
                        }
                }

                // Save final pose for teleop
                PoseStorage.currentPose = drive.localizer.getPose();
        }
}
