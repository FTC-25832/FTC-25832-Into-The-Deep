package org.firstinspires.ftc.teamcode.utils;

/**
 * Controls claw state and sequences, handling debouncing and state management
 */
public class ClawController {
        private final double DEBOUNCE_TIME = 80; // ms
        private boolean isOpen;
        private boolean isGrabSequenceRunning;
        private double lastButtonPressTime;
        private final Runnable openClaw;
        private final Runnable closeClaw;

        public ClawController(Runnable openAction, Runnable closeAction) {
                this.openClaw = openAction;
                this.closeClaw = closeAction;
                this.isOpen = false;
                this.isGrabSequenceRunning = false;
                this.lastButtonPressTime = 0;
        }

        /**
         * Handle manual control, change the claw state
         * @param currentTime   Current system time in ms
         */
        public void handleManualControl(double currentTime) {
                if (!isGrabSequenceRunning) {
                        if (currentTime - lastButtonPressTime > DEBOUNCE_TIME) {
                                isOpen = !isOpen;
                                if (isOpen) {
                                        openClaw.run();
                                } else {
                                        closeClaw.run();
                                }
                        }
                        lastButtonPressTime = currentTime;
                }
        }

        /**
         * Start a grab sequence
         * 
         * @return true if sequence was started, false if already running
         */
        public boolean startGrabSequence() {
                if (!isGrabSequenceRunning) {
                        isGrabSequenceRunning = true;
                        return true;
                }
                return false;
        }

        /**
         * End the grab sequence
         */
        public void endGrabSequence() {
                isGrabSequenceRunning = false;
        }

        /**
         * Check if a grab sequence can be started
         * 
         * @return true if no sequence is running
         */
        public boolean canStartGrabSequence() {
                return !isGrabSequenceRunning;
        }

        /**
         * Get current claw state
         * 
         * @return true if open, false if closed
         */
        public boolean isOpen() {
                return isOpen;
        }

        /**
         * Set claw state directly (used by sequences)
         * 
         * @param open true to open, false to close
         */
        public void setOpen(boolean open) {
                this.isOpen = open;
                if (open) {
                        openClaw.run();
                } else {
                        closeClaw.run();
                }
        }

        /**
         * Check if a grab sequence is running
         * 
         * @return true if sequence is running
         */
        public boolean isGrabSequenceRunning() {
                return isGrabSequenceRunning;
        }
}
