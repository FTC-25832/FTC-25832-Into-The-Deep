package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * executing the grab sequence on the lower slide
 */
public class LowerSlideGrabSequenceCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private final long POS_GRAB_TIMEOUT = ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT;
        private final long CLAW_CLOSE_TIMEOUT = ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT;
        private final long POS_HOVER_TIMEOUT = ConfigVariables.LowerSlideVars.POS_HOVER_TIMEOUT;

        private long startTime;
        private boolean started = false;

        public LowerSlideGrabSequenceCommand(LowerSlide lowSlide) {
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
                        started = true;
                        lowSlide.openClaw();
                        return;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;
                packet.put("grabSequence/elapsed", elapsedTime);

                // Move to grab position after initial delay
                // Close claw after grab position timeout
                // Move to hover position after claw close timeout
                if (elapsedTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT + POS_HOVER_TIMEOUT) {
                        lowSlide.pos_hover();
                } else if (elapsedTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT) {
                        lowSlide.closeClaw();
                } else if (elapsedTime >= POS_GRAB_TIMEOUT) {
                        lowSlide.pos_grab();
                }

        }

        @Override
        public boolean isFinished() {
                if (!started)
                        return false;
                return System.currentTimeMillis() - startTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT
                                + POS_HOVER_TIMEOUT;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        lowSlide.stop();
                }
        }
}
