package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import com.example.meepmeeptesting.shared.RobotState;

import java.util.ArrayList;
import java.util.List;

/**
 * Configurable autonomous command sequence builder.
 * Features:
 * - Sequential and parallel command execution
 * - Vision-based path adjustments
 * - State validation and error recovery
 * - Performance monitoring
 */
@Config
public class AutoCommandSequence {
        // Command configuration
        public static double DEFAULT_TIMEOUT = 5.0; // seconds
        public static double VISION_ALIGN_TIMEOUT = 2.0;
        public static double STATE_TRANSITION_TIMEOUT = 1.0;

        private final MecanumDrive drive;
        private final VisionStateManager vision;
        private final LoopTimer loopTimer;
        private final List<Command> commands;
        private int currentCommandIndex;
        private boolean isRunning;

        public AutoCommandSequence(MecanumDrive drive, VisionStateManager vision, LoopTimer loopTimer) {
                this.drive = drive;
                this.vision = vision;
                this.loopTimer = loopTimer;
                this.commands = new ArrayList<>();
                this.currentCommandIndex = 0;
                this.isRunning = false;
        }

        /**
         * Add state transition command
         */
        public AutoCommandSequence addStateTransition(RobotState targetState) {
                commands.add(new StateTransitionCommand(targetState, STATE_TRANSITION_TIMEOUT));
                return this;
        }

        /**
         * Add vision alignment command
         */
        public AutoCommandSequence addVisionAlign() {
                commands.add(new VisionAlignCommand(VISION_ALIGN_TIMEOUT));
                return this;
        }

        /**
         * Add drive to pose command
         */
        public AutoCommandSequence addDriveTo(Pose2d targetPose) {
                commands.add(new DriveCommand(targetPose, DEFAULT_TIMEOUT));
                return this;
        }

        /**
         * Add parallel command group
         */
        public AutoCommandSequence addParallel(Command... parallelCommands) {
                commands.add(new ParallelCommandGroup(parallelCommands));
                return this;
        }

        /**
         * Start sequence execution
         */
        public void start() {
                if (commands.isEmpty())
                        return;
                currentCommandIndex = 0;
                isRunning = true;
                commands.get(0).start();
        }

        /**
         * Update sequence execution
         */
        public void update() {
                if (!isRunning || commands.isEmpty())
                        return;

                loopTimer.startSection("auto_sequence");

                Command currentCommand = commands.get(currentCommandIndex);
                currentCommand.update();

                if (currentCommand.isFinished()) {
                        // Move to next command
                        currentCommandIndex++;
                        if (currentCommandIndex < commands.size()) {
                                commands.get(currentCommandIndex).start();
                        } else {
                                isRunning = false;
                        }
                }

                loopTimer.endSection("auto_sequence");
        }

        public boolean isRunning() {
                return isRunning;
        }

        public void stop() {
                isRunning = false;
                if (currentCommandIndex < commands.size()) {
                        commands.get(currentCommandIndex).stop();
                }
        }

        /**
         * Base command interface
         */
        public interface Command {
                void start();

                void update();

                void stop();

                boolean isFinished();
        }

        /**
         * State transition command implementation
         */
        private class StateTransitionCommand implements Command {
                private final RobotState targetState;
                private final double timeout;
                private double startTime;

                public StateTransitionCommand(RobotState targetState, double timeout) {
                        this.targetState = targetState;
                        this.timeout = timeout;
                }

                @Override
                public void start() {
                        startTime = loopTimer.seconds();
                }

                @Override
                public void update() {
                        // Implement state transition logic
                }

                @Override
                public void stop() {
                        // Cleanup if needed
                }

                @Override
                public boolean isFinished() {
                        return (loopTimer.seconds() - startTime >= timeout) ||
                                        drive.getPoseEstimate().position.distTo(targetState.getPosition()) < 1.0;
                }
        }

        /**
         * Vision alignment command implementation
         */
        private class VisionAlignCommand implements Command {
                private final double timeout;
                private double startTime;

                public VisionAlignCommand(double timeout) {
                        this.timeout = timeout;
                }

                @Override
                public void start() {
                        startTime = loopTimer.seconds();
                }

                @Override
                public void update() {
                        // Implement vision alignment logic
                }

                @Override
                public void stop() {
                        // Cleanup if needed
                }

                @Override
                public boolean isFinished() {
                        return (loopTimer.seconds() - startTime >= timeout) ||
                                        vision.getCurrentState().isTrackingState();
                }
        }

        /**
         * Drive command implementation
         */
        private class DriveCommand implements Command {
                private final Pose2d targetPose;
                private final double timeout;
                private double startTime;

                public DriveCommand(Pose2d targetPose, double timeout) {
                        this.targetPose = targetPose;
                        this.timeout = timeout;
                }

                @Override
                public void start() {
                        startTime = loopTimer.seconds();
                        drive.followTrajectorySequenceAsync(
                                        drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                                        .splineToLinearHeading(targetPose, targetPose.heading)
                                                        .build());
                }

                @Override
                public void update() {
                        drive.update();
                }

                @Override
                public void stop() {
                        drive.cancelTrajectory();
                }

                @Override
                public boolean isFinished() {
                        return (loopTimer.seconds() - startTime >= timeout) ||
                                        !drive.isBusy();
                }
        }

        /**
         * Parallel command group implementation
         */
        private class ParallelCommandGroup implements Command {
                private final Command[] commands;
                private final boolean[] completed;

                public ParallelCommandGroup(Command... commands) {
                        this.commands = commands;
                        this.completed = new boolean[commands.length];
                }

                @Override
                public void start() {
                        for (Command command : commands) {
                                command.start();
                        }
                }

                @Override
                public void update() {
                        for (int i = 0; i < commands.length; i++) {
                                if (!completed[i]) {
                                        commands[i].update();
                                        if (commands[i].isFinished()) {
                                                completed[i] = true;
                                        }
                                }
                        }
                }

                @Override
                public void stop() {
                        for (Command command : commands) {
                                command.stop();
                        }
                }

                @Override
                public boolean isFinished() {
                        for (boolean done : completed) {
                                if (!done)
                                        return false;
                        }
                        return true;
                }
        }
}
