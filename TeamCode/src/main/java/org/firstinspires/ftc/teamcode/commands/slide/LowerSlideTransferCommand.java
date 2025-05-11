package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;

public class LowerSlideTransferCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private boolean started = false;
        private long startTime = 0;
        private static final long SERVO_DELAY = 400;

        public LowerSlideTransferCommand(LowerSlide lowSlide) {
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
                        startTime = System.currentTimeMillis();
                        lowSlide.openClaw();
                        started = true;
                        packet.put("transferAction", "Started");
                        return;
                }

                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("transferElapsed", elapsed);
                packet.put("transferComplete", elapsed >= SERVO_DELAY);
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
                        lowSlide.stop();
                }
        }
}
