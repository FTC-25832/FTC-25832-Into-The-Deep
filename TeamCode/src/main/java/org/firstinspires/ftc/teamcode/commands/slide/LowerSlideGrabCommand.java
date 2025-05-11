package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;

/**
 * execute the grab sequence for the lower slide.
 */
public class LowerSlideGrabCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private boolean started = false;
        private long startTime = 0;
        private int step = 0;
        private static final long SERVO_DELAY = 400;

        public LowerSlideGrabCommand(LowerSlide lowSlide) {
                this.lowSlide = lowSlide;
                addRequirement(lowSlide);
        }

        @Override
        public void initialize() {
                started = false;
                step = 0;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                if (!started) {
                        startTime = System.currentTimeMillis();
                        lowSlide.pos_grab();
                        started = true;
                        packet.put("grabAction", "Started");
                        return;
                }

                long elapsed = System.currentTimeMillis() - startTime;
                packet.put("grabStep", step);
                packet.put("grabElapsed", elapsed);

                switch (step) {
                        case 0:
                                if (elapsed > SERVO_DELAY) {
                                        lowSlide.openClaw();
                                        startTime = System.currentTimeMillis();
                                        step++;
                                        packet.put("grabState", "Opening claw");
                                }
                                break;
                        case 1:
                                if (elapsed > SERVO_DELAY) {
                                        lowSlide.closeClaw();
                                        startTime = System.currentTimeMillis();
                                        step++;
                                        packet.put("grabState", "Closing claw");
                                }
                                break;
                        case 2:
                                if (elapsed > SERVO_DELAY) {
                                        lowSlide.pos_up();
                                        startTime = System.currentTimeMillis();
                                        step++;
                                        packet.put("grabState", "Moving up");
                                }
                                break;
                        case 3:
                                packet.put("grabState", "Finishing movement");
                                if (elapsed > SERVO_DELAY) {
                                        step++;
                                }
                                break;
                        default:
                                packet.put("grabState", "Complete");
                }
        }

        @Override
        public boolean isFinished() {
                return started && step >= 4;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        lowSlide.stop();
                }
        }
}
