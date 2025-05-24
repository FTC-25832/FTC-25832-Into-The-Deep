package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.ClawController; // Import ClawController
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * executing the grab sequence on the upper slide
 */
public class UpperSlideGrabSequenceCommand extends CommandBase {
        private final UpperSlide upSlide;
        private final ClawController clawController; // Add ClawController field

        // Using placeholder timeouts similar to LowerSlide as UpperSlideVars were commented out in original
        // TODO: Define these in ConfigVariables.UpperSlideVars if not already present
        private final long POS_GRAB_TIMEOUT = 500; 
        private final long CLAW_CLOSE_TIMEOUT = 500;
        private final long POS_HOVER_TIMEOUT = 500; 

        private long startTime;
        private boolean started = false;
        private boolean canExecute = false; // To control execution based on ClawController

        public UpperSlideGrabSequenceCommand(UpperSlide upSlide, ClawController clawController) {
                this.upSlide = upSlide;
                this.clawController = clawController; // Store the ClawController
                addRequirement(upSlide);
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
                        upSlide.openClaw(); // Open claw at the beginning of actual execution
                        started = true;
                        return; 
                }

                long elapsedTime = System.currentTimeMillis() - startTime;
                packet.put("grabSequence/elapsed", elapsedTime);

                if (elapsedTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT + POS_HOVER_TIMEOUT) {
                        upSlide.pos_hover();
                } else if (elapsedTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT) {
                        upSlide.closeClaw();
                } else if (elapsedTime >= POS_GRAB_TIMEOUT) {
                        upSlide.pos_grab();
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
                return System.currentTimeMillis() - startTime >= POS_GRAB_TIMEOUT + CLAW_CLOSE_TIMEOUT + POS_HOVER_TIMEOUT;
        }

        @Override
        public void end(boolean interrupted) {
                if (canExecute) { 
                    clawController.endGrabSequence();
                }
                if (interrupted && canExecute) { 
                    upSlide.stop();
                }
        }
}
