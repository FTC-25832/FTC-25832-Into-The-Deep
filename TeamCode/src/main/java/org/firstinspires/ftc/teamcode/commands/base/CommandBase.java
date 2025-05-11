package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Base class for all commands
 */
public abstract class CommandBase implements Command {
        private final Set<SubsystemBase> requirements = new HashSet<>();

        /**
         * Gets the subsystems required by this command
         */
        public Set<SubsystemBase> getRequirements() {
                return requirements;
        }

        /**
         * Adds subsystem requirements
         */
        protected void addRequirement(SubsystemBase... requirements) {
            Collections.addAll(this.requirements, requirements);
        }

        @Override
        public void initialize() {
        }

        @Override
        public void execute(TelemetryPacket packet) {
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
        }

        @Override
        public Action toAction() {
                return Command.super.toAction();
        }
}
