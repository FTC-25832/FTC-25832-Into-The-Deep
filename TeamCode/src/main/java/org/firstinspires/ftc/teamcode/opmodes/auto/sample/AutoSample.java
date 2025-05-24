package org.firstinspires.ftc.teamcode.opmodes.auto.sample;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action; // Will be removed from method signatures but used locally
import com.acmerobotics.roadrunner.ParallelAction; // To be replaced by ParallelCommandGroup
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction; // To be replaced by SequentialCommandGroup
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions; // To be removed
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// HardwareMap and Robot are not directly used after refactor, but TelemetryPacket is.

import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.commands.base.CommandScheduler; // Added
import org.firstinspires.ftc.teamcode.commands.base.ParallelCommandGroup; // Added
import org.firstinspires.ftc.teamcode.commands.base.RoadrunnerActionCommand; // Added
import org.firstinspires.ftc.teamcode.commands.base.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.base.WaitCommand; // Added
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideGrabSequenceCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideUpdatePID;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideUpdatePID;
// Vision commands might not be used in this specific sample's direct sequence, but keep for now
// import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustAutoCommand;
// import org.firstinspires.ftc.teamcode.commands.vision.AngleAdjustCommand;
// import org.firstinspires.ftc.teamcode.commands.vision.DistanceAdjustCommand;
// import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive; // No longer a direct field
import org.firstinspires.ftc.teamcode.subsystems.drive.DrivetrainSubsystem; // Added
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight; // Keep if vision commands are used elsewhere or planned
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

@Autonomous
public final class AutoSample extends LinearOpMode {
        // private MecanumDrive drive; // Removed
        private DrivetrainSubsystem drivetrainSubsystem; // Added
        private CommandScheduler scheduler; // Added

        private LowerSlide lowSlide;
        private LowerSlideCommands lowerSlideCommands;
        private UpperSlide upSlide;
        private UpperSlideCommands upperSlideCommands;
        // Limelight camera; // Declare if vision commands that need it are used

        private Command waitSeconds(double seconds) { // Signature changed
                return new WaitCommand(seconds); // Assumes WaitCommand constructor takes double seconds
        }

        private Command scoreSequence(RobotPosition startPOS, double lowerslideExtendLength) {
                Action driveToAction = drivetrainSubsystem.drive.actionBuilder(startPOS.pose)
                                .strafeToLinearHeading(SCORE.pos, SCORE.heading)
                                .build();
                Command driveToScoreCmd = new RoadrunnerActionCommand(driveToAction, drivetrainSubsystem);

                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                driveToScoreCmd,
                                                upperSlideCommands.closeClaw(),
                                                upperSlideCommands.slidePos3()
                                ),
                                upperSlideCommands.front(),
                                waitSeconds(ConfigVariables.AutoTesting.A_DROPDELAY_S),
                                new ParallelCommandGroup(
                                                upperSlideCommands.openClaw(),
                                                lowerSlideCommands.hover(),
                                                lowerSlideCommands.setSlidePos(lowerslideExtendLength)
                                ),
                                waitSeconds(ConfigVariables.AutoTesting.B_DROPPEDAFTERDELAY_S),
                                upperSlideCommands.scorespec()
                );
        }

        private Command pickupAndScoreSequence(RobotPosition startPOS, AutoPaths.RobotPosition pickupPos,
                        double lowerslideExtendLength) {
                Action driveToPickupAction = drivetrainSubsystem.drive.actionBuilder(startPOS.pose)
                                .strafeToLinearHeading(pickupPos.pos, pickupPos.heading)
                                .build();
                Command driveToPickupCmd = new RoadrunnerActionCommand(driveToPickupAction, drivetrainSubsystem);

                return new SequentialCommandGroup(
                                driveToPickupCmd,
                                new ParallelCommandGroup(
                                                upperSlideCommands.slidePos0(),
                                                // Assuming LowerSlideGrabSequenceCommand can handle null ClawController for Auto
                                                // or its specific ClawController logic is not critical here.
                                                new LowerSlideGrabSequenceCommand(lowSlide, null) 
                                ),
                                waitSeconds(ConfigVariables.AutoTesting.C_AFTERGRABDELAY_S),
                                lowerSlideCommands.slidePos0(),
                                waitSeconds(ConfigVariables.AutoTesting.D_SLIDEPOS0AFTERDELAY_S),
                                lowerSlideCommands.up(),
                                waitSeconds(ConfigVariables.AutoTesting.E_LOWSLIDEUPAFTERDELAY_S),
                                upperSlideCommands.transfer(),
                                waitSeconds(ConfigVariables.AutoTesting.F_TRANSFERAFTERDELAY_S),
                                upperSlideCommands.closeClaw(),
                                waitSeconds(ConfigVariables.AutoTesting.G_LOWSLIDETRANSFEROPENCLAWAFTERDELAY_S),
                                lowerSlideCommands.openClaw(),
                                waitSeconds(ConfigVariables.AutoTesting.H_TRANSFERCOMPLETEAFTERDELAY_S),
                                scoreSequence(pickupPos, lowerslideExtendLength) // This now returns a Command
                );
        }

        @Override
        public void runOpMode() throws InterruptedException {
                // Initialize CommandScheduler
                scheduler = CommandScheduler.getInstance();
                scheduler.reset(); // Clear state from previous OpModes

                // Initialize subsystems
                lowSlide = new LowerSlide();
                upSlide = new UpperSlide();
                lowSlide.initialize(hardwareMap); // Assuming this properly sets up hardware
                upSlide.initialize(hardwareMap);   // Assuming this properly sets up hardware
                drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, START.pose);

                // Register subsystems
                scheduler.registerSubsystem(lowSlide);
                scheduler.registerSubsystem(upSlide);
                scheduler.registerSubsystem(drivetrainSubsystem);

                // Initialize command factories (these should return Command instances)
                lowerSlideCommands = new LowerSlideCommands(lowSlide);
                upperSlideCommands = new UpperSlideCommands(upSlide);

                // Initial Setup Command
                Command initialSetup = new SequentialCommandGroup(
                                lowerSlideCommands.up(),
                                upperSlideCommands.scorespec(),
                                upperSlideCommands.closeClaw()
                );
                scheduler.schedule(initialSetup);

                // Wait for this initial setup to complete before waitForStart()
                while (opModeInInit() && !initialSetup.isFinished() && !isStopRequested()) {
                    scheduler.run(new TelemetryPacket()); // Pass a dummy packet
                    // telemetry.update(); // Optional: update telemetry during init
                }
                
                if(isStopRequested()) return;

                if(!initialSetup.isFinished()) {
                    telemetry.addLine("Initial setup did not complete.");
                    telemetry.update();
                    // Ending the command if it didn't finish, to prevent issues.
                    initialSetup.end(true); 
                }

                // Define the main autonomous sequence command
                Command fullAutonomousCommand = new ParallelCommandGroup(
                                new LowerSlideUpdatePID(lowSlide), // Assuming these are persistent Commands
                                new UpperSlideUpdatePID(upSlide),   // that run in their execute()
                                new SequentialCommandGroup(
                                                upperSlideCommands.scorespec(),
                                                scoreSequence(START, ConfigVariables.AutoTesting.Z_LowerslideExtend_FIRST),
                                                pickupAndScoreSequence(SCORE, PICKUP1, ConfigVariables.AutoTesting.Z_LowerslideExtend_SECOND),
                                                pickupAndScoreSequence(SCORE, PICKUP2, ConfigVariables.AutoTesting.Z_LowerslideExtend_THIRD),
                                                lowerSlideCommands.setSpinClawDeg(ConfigVariables.LowerSlideVars.ZERO + 90),
                                                pickupAndScoreSequence(SCORE, PICKUP3, 0),
                                                upperSlideCommands.setSlidePos(0),
                                                lowerSlideCommands.up(),
                                                upperSlideCommands.front(),
                                                lowerSlideCommands.slidePos0(),
                                                upperSlideCommands.slidePos0(),
                                                new RoadrunnerActionCommand( // Drive to TeleOp start
                                                                drivetrainSubsystem.drive.actionBuilder(SCORE.pose)
                                                                                .turnTo(TELEOP_START.heading)
                                                                                .build(),
                                                                drivetrainSubsystem
                                                ),
                                                new RoadrunnerActionCommand(
                                                                drivetrainSubsystem.drive.actionBuilder(new Pose2d(SCORE.pos, TELEOP_START.heading))
                                                                                .strafeToLinearHeading(TELEOP_START.pos, TELEOP_START.heading)
                                                                                .build(),
                                                                drivetrainSubsystem
                                                )
                                )
                );

                waitForStart();
                if (isStopRequested()) return;

                scheduler.schedule(fullAutonomousCommand);

                while (opModeIsActive() && !isStopRequested() && !fullAutonomousCommand.isFinished()) {
                    TelemetryPacket packet = new TelemetryPacket();
                    scheduler.run(packet);
                    // telemetry.addData("X", drivetrainSubsystem.drive.pose.position.x);
                    // telemetry.addData("Y", drivetrainSubsystem.drive.pose.position.y);
                    // telemetry.update();
                    // FtcDashboard.getInstance().sendTelemetryPacket(packet); // If using dashboard
                }

                // Cleanup and save final pose
                // Note: drive.pose is used here, ensure DrivetrainSubsystem.drive is accessible
                // or provide a getter like drivetrainSubsystem.getCurrentPose()
                PoseStorage.currentPose = drivetrainSubsystem.drive.pose; 
                scheduler.cancelAll(); // Cancel all commands
                // scheduler.reset(); // Optional: Fully reset scheduler if desired
        }
}
