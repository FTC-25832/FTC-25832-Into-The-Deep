package org.firstinspires.ftc.teamcode.commands.base;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Set;

/**
 * Base interface for all commands
 */
public interface Command extends ActionConvertable {
        /**
         * Initialize the command
         */
        void initialize();

        /**
         * Execute one time step of the command
         */
        void execute(TelemetryPacket packet);

        /**
         * Returns true when the command should end
         */
        boolean isFinished();

        /**
         * Called when the command ends
         */
        void end(boolean interrupted);

        /**
         * Gets the subsystems required by this command
         */
        Set<SubsystemBase> getRequirements();

        /**
         * Gets the timeout for this command in milliseconds
         * 
         * @return timeout in ms, or 0 for no timeout
         */
        default long getTimeout() {
                return 0;
        }

        /**
         * Convert command to RoadRunner Action
         */
        @Override
        default Action toAction() {
                return new Action() {
                        private boolean initialized = false;

                        @Override
                        public boolean run(@NonNull TelemetryPacket packet) {
                                if (!initialized) {
                                        initialize();
                                        initialized = true;
                                }
                                if (isFinished()) {
                                        end(false);
                                        return false;
                                }
                                execute(packet);

                                if (!isFinished()) {
                                        return true;
                                }

                                end(false);
                                return false;
                        }
                };
        }
}
