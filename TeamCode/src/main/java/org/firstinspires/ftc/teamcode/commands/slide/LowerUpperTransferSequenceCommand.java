package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.commands.slide.LowerSlideCommands;
import org.firstinspires.ftc.teamcode.commands.slide.UpperSlideCommands;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * Command for the lower+upper slide transfer sequence (auto/teleop)
 */
public class LowerUpperTransferSequenceCommand extends CommandBase {
        private final LowerSlideCommands lowerSlideCommands;
        private final UpperSlideCommands upperSlideCommands;
        private int step = 0;
        private long stepStartTime = 0;
        private boolean started = false;

        public LowerUpperTransferSequenceCommand(LowerSlideCommands lowerSlideCommands,
                        UpperSlideCommands upperSlideCommands) {
                this.lowerSlideCommands = lowerSlideCommands;
                this.upperSlideCommands = upperSlideCommands;
        }

        @Override
        public void initialize() {
                step = 0;
                started = false;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                long now = System.currentTimeMillis();
                if (!started) {
                        stepStartTime = now;
                        started = true;
                        lowerSlideCommands.slidePos2().run(packet);
                        packet.put("transferSequence/step", "slidePos2");
                        return;
                }
                long elapsed = now - stepStartTime;
                switch (step) {
                        // use numbers or text as step terms?
                        case 0:
                                lowerSlideCommands.slidePos2().run(packet);
                                step = 1;
                                stepStartTime = now;
                                packet.put("transferSequence/step", "lowerslide pos 2");

                                break;
                        case 1:
                                if (elapsed >= (long) (ConfigVariables.AutoTesting.D_SLIDEPOS0AFTERDELAY_S * 1000)) {
                                        lowerSlideCommands.up().run(packet);
                                        step = 2;
                                        stepStartTime = now;
                                        packet.put("transferSequence/step", "up");
                                }
                                break;
                        case 2:
                                if (elapsed >= (long) (ConfigVariables.AutoTesting.E_LOWSLIDEUPAFTERDELAY_S * 1000)) {
                                        upperSlideCommands.transfer().run(packet);
                                        step = 3;
                                        stepStartTime = now;
                                        packet.put("transferSequence/step", "transfer");
                                }
                                break;
                        case 3:
                                if (elapsed >= (long) (ConfigVariables.AutoTesting.F_TRANSFERAFTERDELAY_S * 1000)) {
                                        upperSlideCommands.closeClaw().run(packet);
                                        step = 4;
                                        stepStartTime = now;
                                        packet.put("transferSequence/step", "closeClaw");
                                }
                                break;
                        case 4:
                                if (elapsed >= (long) (ConfigVariables.AutoTesting.G_LOWSLIDETRANSFEROPENCLAWAFTERDELAY_S
                                                * 1000)) {
                                        lowerSlideCommands.openClaw().run(packet);
                                        step = 5;
                                        packet.put("transferSequence/step", "openClaw");
                                }
                                break;
                }
        }

        @Override
        public boolean isFinished() {
                return started && step == 5;
        }

        @Override
        public void end(boolean interrupted) {
                // No special cleanup needed
        }
}