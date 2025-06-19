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
        private long startTime;
        private boolean started = false;
        private int step = 0;

        public LowerSlideGrabSequenceCommand(LowerSlide lowSlide) {
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
                long now = System.currentTimeMillis();
                if (!started) {
                        startTime = now;
                        started = true;
                        step = 0;
                        lowSlide.openClaw();
                        packet.put("grabSequence/step", "openClaw");
                        return;
                }
                long elapsed = now - startTime;
                packet.put("grabSequence/elapsed", elapsed);
                switch (step) {
                        // use numbers or text as step terms?
                        case 0:
                                if (elapsed >= 0) { // openClaw is instant, move to grab after 0ms
                                        lowSlide.setPart1Position(ConfigVariables.LowerSlideVars.GRAB_BIG);
                                        lowSlide.setPart2Position(ConfigVariables.LowerSlideVars.GRAB_SMALL);
                                        step = 1;
                                        startTime = now;
                                        packet.put("grabSequence/step", "grabPos");
                                }
                                break;
                        case 1:
                                if (elapsed >= ConfigVariables.LowerSlideVars.POS_GRAB_TIMEOUT) {
                                        lowSlide.closeClaw();
                                        step = 2;
                                        startTime = now;
                                        packet.put("grabSequence/step", "closeClaw");
                                }
                                break;
                        case 2:
                                if (elapsed >= ConfigVariables.LowerSlideVars.CLAW_CLOSE_TIMEOUT) {
                                        lowSlide.setPart1Position(ConfigVariables.LowerSlideVars.HOVER_BIG);
                                        lowSlide.setPart2Position(ConfigVariables.LowerSlideVars.HOVER_SMALL);
                                        step = 3;
                                        startTime = now;
                                        packet.put("grabSequence/step", "hover");
                                }
                                break;
                        case 3:
                                if (elapsed >= ConfigVariables.LowerSlideVars.POS_HOVER_TIMEOUT) {
                                        step = 4;
                                        packet.put("grabSequence/step", "done");
                                }
                                break;
                }
        }

        @Override
        public boolean isFinished() {
                return started && step == 4;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        lowSlide.stop();
                }
        }
}