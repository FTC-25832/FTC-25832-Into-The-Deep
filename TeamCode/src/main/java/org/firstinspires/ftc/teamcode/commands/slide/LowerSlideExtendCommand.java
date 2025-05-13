package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;

/**
 * extend to target position.
 */
public class LowerSlideExtendCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private boolean started = false;
        private static final double SLIDE_TOLERANCE = 50;
        private static final double CM_TO_TICKS = (28.0 * 4.0) / (37.0 * 3.14) * 10;
        private final double targetPosition = 34.0 * CM_TO_TICKS;

        public LowerSlideExtendCommand(LowerSlide lowSlide) {
                this.lowSlide = lowSlide;
                addRequirement(lowSlide);
        }

        @Override
        public void initialize() {
                started = false;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                if (!started) {
                        lowSlide.setPositionCM(34.0);
                        started = true;
                        packet.put("slideAction", "Extend started");
                        packet.put("targetPosition", targetPosition);
                        return;
                }

                double currentPos = lowSlide.getCurrentPosition();
                packet.put("slideCurrentPos", currentPos);
                packet.put("slideTargetPos", targetPosition);
                packet.put("slideDelta", Math.abs(currentPos - targetPosition));
                packet.put("slideMoving", !isFinished());
        }

        @Override
        public boolean isFinished() {
                if (!started)
                        return false;
                double currentPos = lowSlide.getCurrentPosition();
                return Math.abs(currentPos - targetPosition) <= SLIDE_TOLERANCE;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        lowSlide.stop();
                }
        }
}
