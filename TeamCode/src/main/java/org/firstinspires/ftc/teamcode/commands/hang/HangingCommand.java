package org.firstinspires.ftc.teamcode.commands.hang;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.hang.Hanging;

public class HangingCommand extends CommandBase {
        private final Hanging hanging;
        private final Direction direction;
        private boolean isFinished = false;

        public enum Direction {
                FORWARD,
                BACKWARD,
                STOP
        }

        /**
         * @param hanging   The hanging subsystem
         * @param direction Direction to move (forward, backward, or stop)
         */
        public HangingCommand(Hanging hanging, Direction direction) {
                this.hanging = hanging;
                this.direction = direction;
                addRequirement(hanging);
        }

        @Override
        public void initialize() {
                switch (direction) {
                        case FORWARD:
                                hanging.turnForward();
                                break;
                        case BACKWARD:
                                hanging.turnBackward();
                                break;
                        case STOP:
                                hanging.stop();
                                break;
                }
                isFinished = true;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Command completes immediately after setting direction
        }

        @Override
        public boolean isFinished() {
                return isFinished;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        hanging.stop();
                }
        }
}
