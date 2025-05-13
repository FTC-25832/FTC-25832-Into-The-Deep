package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

/**
 * to a specific position
 */
public class UpperSlidePositionCommand extends CommandBase {
        private final UpperSlide upSlide;
        private final double targetPositionCm;
        private final double armPosition;
        private final double swingPosition;

        /**
         * @param upSlide          The upper slide subsystem
         * @param targetPositionCm Target position in centimeters
         * @param armPosition      Target arm position (0-1)
         * @param swingPosition    Target swing position (0-1)
         */
        public UpperSlidePositionCommand(UpperSlide upSlide, double targetPositionCm, double armPosition,
                        double swingPosition) {
                this.upSlide = upSlide;
                this.targetPositionCm = targetPositionCm;
                this.armPosition = armPosition;
                this.swingPosition = swingPosition;
                addRequirement(upSlide);
        }

        @Override
        public void initialize() {
                upSlide.setPositionCM(targetPositionCm);
                upSlide.setArmPosition(armPosition);
                upSlide.setSwingPosition(swingPosition);
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Run PID update to move motors
                double power = upSlide.updatePID();

                // Add telemetry
                double error = Math.abs(upSlide.pidfController.destination - upSlide.getCurrentPosition());
                packet.put("upperSlidePosition/error", error);
                packet.put("upperSlidePosition/target", targetPositionCm);
                packet.put("upperSlidePosition/power", power);
        }

        @Override
        public boolean isFinished() {
                // Consider the command finished when we're within 50 encoder ticks of target
                return Math.abs(upSlide.pidfController.destination - upSlide.getCurrentPosition()) < 50;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        upSlide.stop();
                }
        }
}
