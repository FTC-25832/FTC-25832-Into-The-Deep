package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.ClawController; // Import ClawController
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * executing the grab sequence on the lower slide
 */
public class LowerSlideGrabSequenceCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private final ClawController clawController; // Add ClawController field
        private final long POS_GRAB_TIMEOUT = ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT;
        private final long CLAW_CLOSE_TIMEOUT = ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT;
        private final long POS_HOVER_TIMEOUT = ConfigVariables.LowerSlideVars.POS_HOVER_TIMEOUT;

        private long startTime;
        private boolean started = false;
        private boolean canExecute = false; // To control execution

        public LowerSlideGrabSequenceCommand(LowerSlide lowSlide, ClawController clawController) {
                this.lowSlide = lowSlide;
                this.clawController = clawController; // Store the ClawController
                addRequirement(lowSlide);
        }

        @Override
        public void initialize() {
            if (clawController.canStartGrabSequence()) {
                clawController.startGrabSequence();
                canExecute = true;
                started = false;
            } else {
                canExecute = false;
            }
        }

        @Override
        public void execute(TelemetryPacket packet) {
            if (!canExecute) {
                return; 
            }

            if (!started) {
                startTime = System.currentTimeMillis();
                lowSlide.openClaw(); // Open claw at the beginning of actual execution
                started = true;
                return; 
            }

            long elapsedTime = System.currentTimeMillis() - startTime;
            packet.put("grabSequence/elapsed", elapsedTime);

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
                if (!canExecute) {
                    return true; 
                }
                if (!started) {
                    return false; 
                }
                return System.currentTimeMillis() - startTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT
                                + POS_HOVER_TIMEOUT;
        }

        @Override
        public void end(boolean interrupted) {
                if (canExecute) { 
                    clawController.endGrabSequence();
                }
                if (interrupted && canExecute) { 
                    lowSlide.stop();
                }
        }
}
