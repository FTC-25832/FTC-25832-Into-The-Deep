package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;

/**
 * to a specific position and configuration
 */
public class LowerSlidePositionCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private final double targetPositionCm;
        private final double part1Position;
        private final double part2Position;
        private final double spinClawDegrees;

        /**
         * @param lowSlide         The lower slide subsystem
         * @param targetPositionCm Target position in centimeters
         * @param part1Position    Target position for part1 servo (0-1)
         * @param part2Position    Target position for part2 servo (0-1)
         * @param spinClawDegrees  Target angle for spinclaw in degrees (0-270)
         */
        public LowerSlidePositionCommand(LowerSlide lowSlide, double targetPositionCm, double part1Position,
                        double part2Position, double spinClawDegrees) {
                this.lowSlide = lowSlide;
                this.targetPositionCm = targetPositionCm;
                this.part1Position = part1Position;
                this.part2Position = part2Position;
                this.spinClawDegrees = spinClawDegrees;
                addRequirement(lowSlide);
        }

        @Override
        public void initialize() {
                lowSlide.setPositionCM(targetPositionCm);
                lowSlide.setPart1Position(part1Position);
                lowSlide.setPart2Position(part2Position);
                lowSlide.spinclawSetPositionDeg(spinClawDegrees);
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Run PID update to move motor
                double power = lowSlide.updatePID();

                // Add telemetry
                double error = Math.abs(lowSlide.pidController.destination - lowSlide.getCurrentPosition());
                packet.put("lowerSlidePosition/error", error);
                packet.put("lowerSlidePosition/target", targetPositionCm);
                packet.put("lowerSlidePosition/power", power);
        }

        @Override
        public boolean isFinished() {
                // Consider the command finished when we're within 50 encoder ticks of target
                return Math.abs(lowSlide.pidController.destination - lowSlide.getCurrentPosition()) < 50;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        lowSlide.stop();
                }
        }
}
