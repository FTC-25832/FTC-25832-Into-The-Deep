package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

public class ClawToggleCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private final UpperSlide upSlide;
        private final boolean isUpper;
        private final boolean targetOpen;

        /**
         * @param lowSlide   The lower slide subsystem
         * @param upSlide    The upper slide subsystem
         * @param isUpper    True to control upper claw, false for lower claw
         * @param targetOpen True to open the claw, false to close it
         */
        public ClawToggleCommand(LowerSlide lowSlide, UpperSlide upSlide, boolean isUpper, boolean targetOpen) {
                this.lowSlide = lowSlide;
                this.upSlide = upSlide;
                this.isUpper = isUpper;
                this.targetOpen = targetOpen;
                addRequirement(isUpper ? upSlide : lowSlide);
        }

        @Override
        public void initialize() {
                if (isUpper) {
                        if (targetOpen)
                                upSlide.openClaw();
                        else
                                upSlide.closeClaw();
                } else {
                        if (targetOpen)
                                lowSlide.openClaw();
                        else
                                lowSlide.closeClaw();
                }
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Nothing to do during execution
        }

        @Override
        public boolean isFinished() {
                // Command completes immediately after setting claw state
                return true;
        }
}
