package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

/**
 * Base class for slide commands that use PID control
 */
public abstract class SlideCommand implements Command {
        protected boolean initialized = false;

        @Override
        public void initialize() {
                setTargetPosition();
        }

        @Override
        public void execute() {
                updatePID();
        }

        @Override
        public boolean isFinished() {
                return Math.abs(getCurrentPosition() - getTargetPosition()) < 50;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
                if (!initialized) {
                        initialize();
                        initialized = true;
                }
                execute();
                packet.put(getTelemetryName() + "/position", getCurrentPosition());
                packet.put(getTelemetryName() + "/target", getTargetPosition());
                return !isFinished();
        }

        protected abstract void setTargetPosition();

        protected abstract void updatePID();

        protected abstract double getCurrentPosition();

        protected abstract double getTargetPosition();

        protected abstract String getTelemetryName();
}
