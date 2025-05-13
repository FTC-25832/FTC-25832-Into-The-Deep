package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * Base class for slide commands that use PID control
 */
public abstract class SlideCommand implements Action {
        protected boolean initialized = false;

        public void initialize() {
                setTargetPosition();
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

        protected void execute() {
                updatePID();
        }

        protected boolean isFinished() {
                return Math.abs(getCurrentPosition() - getTargetPosition()) < ConfigVariables.General.DISTANCE_THRESHOLD_ENCODER;
        }

        protected abstract void setTargetPosition();

        protected abstract void updatePID();

        protected abstract double getCurrentPosition();

        protected abstract double getTargetPosition();

        protected abstract String getTelemetryName();
}
