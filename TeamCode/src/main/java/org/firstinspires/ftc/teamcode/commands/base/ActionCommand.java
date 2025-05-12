package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.HashSet;
import java.util.Set;

/**
 * Adapter class that wraps a RoadRunner Action into a Command
 */
public class ActionCommand implements Command {
        private final Action action;
        private boolean isFinished = false;

        public ActionCommand(Action action) {
                this.action = action;
        }

        @Override
        public void initialize() {
                // Nothing to initialize
        }

        @Override
        public void execute(TelemetryPacket packet) {
                isFinished = !action.run(packet);
        }

        @Override
        public void end(boolean interrupted) {
                // Nothing to clean up
        }

        @Override
        public boolean isFinished() {
                return isFinished;
        }

        @Override
        public Set<SubsystemBase> getRequirements() {
                // Actions don't have requirements in the same way Commands do
                return new HashSet<>();
        }

        @Override
        public long getTimeout() {
                return 0; // No timeout by default
        }
}
