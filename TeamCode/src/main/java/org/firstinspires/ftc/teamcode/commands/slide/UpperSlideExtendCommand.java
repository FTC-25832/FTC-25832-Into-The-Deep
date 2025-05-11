package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

/**
 * extend the upper slide to its target position.
 */
public class UpperSlideExtendCommand extends CommandBase {
        private final UpperSlide upSlide;
        private boolean started = false;
        private long startTime = 0;
        private static final long SERVO_DELAY = 400;

        public UpperSlideExtendCommand(UpperSlide upSlide) {
                this.upSlide = upSlide;
                addRequirement(upSlide);
        }

        @Override
        public void initialize() {
                started = false;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                if (!started) {
                        upSlide.pos2();
                        upSlide.front();
                        started = true;
                        startTime = System.currentTimeMillis();
                        packet.put("upslideAction", "Extend started");
                        return;
                }

                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("upslideElapsed", elapsed);
                packet.put("upslideComplete", elapsed >= SERVO_DELAY);
        }

        @Override
        public boolean isFinished() {
                if (!started)
                        return false;
                return System.currentTimeMillis() - startTime >= SERVO_DELAY;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        upSlide.stop();
                }
        }
}
