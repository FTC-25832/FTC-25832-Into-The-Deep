package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

/**
 * receive transfer from the lower slide.
 */
public class UpperSlideTransferReceiveCommand extends CommandBase {
        private final UpperSlide upSlide;
        private boolean started = false;
        private long startTime = 0;
        private static final long SERVO_DELAY = 400;

        public UpperSlideTransferReceiveCommand(UpperSlide upSlide) {
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
                        upSlide.closeClaw();
                        started = true;
                        startTime = System.currentTimeMillis();
                        packet.put("upslideAction", "TransferReceive started");
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
