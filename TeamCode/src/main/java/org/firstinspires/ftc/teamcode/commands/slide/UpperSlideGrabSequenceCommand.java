package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * executing the grab sequence on the upper slide
 */
public class UpperSlideGrabSequenceCommand extends CommandBase {
        private final UpperSlide upSlide;
//        private final long POS_GRAB_TIMEOUT = ConfigVariables.UpperSlideVars.POS_GRAB_TIMEOUT;
//        private final long CLAW_CLOSE_TIMEOUT = ConfigVariables.UpperSlideVars.CLAW_CLOSE_TIMEOUT;
//        private final long POS_HOVER_TIMEOUT = ConfigVariables.UpperSlideVars.POS_HOVER_TIMEOUT;

        private long startTime;
        private boolean started = false;

        public UpperSlideGrabSequenceCommand(UpperSlide upSlide) {
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
                        startTime = System.currentTimeMillis();
                        started = true;
                        upSlide.openClaw();
                        return;
                }

                long elapsedTime = System.currentTimeMillis() - startTime;
                packet.put("grabSequence/elapsed", elapsedTime);

                // Move to grab position after initial delay
                // Close claw after grab position timeout
                // Move to hover position after claw close timeout
//                if (elapsedTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT + POS_HOVER_TIMEOUT) {
//                        upSlide.pos_hover();
//                } else if (elapsedTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT) {
//                        upSlide.closeClaw();
//                } else if (elapsedTime >= POS_GRAB_TIMEOUT) {
//                        upSlide.pos_grab();
//                }
        }

        @Override
        public boolean isFinished() {
                if (!started)
                        return false;
                return System.currentTimeMillis() - startTime >= 0;
//                        POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT
//                                + POS_HOVER_TIMEOUT;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        upSlide.stop();
                }
        }
}
