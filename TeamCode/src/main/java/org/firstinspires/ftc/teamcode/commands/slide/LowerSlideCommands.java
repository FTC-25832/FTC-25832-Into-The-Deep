package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables.LowerSlideVars;

/**
 * Factory class for creating lower slide commands
 */
public class LowerSlideCommands {
        private final LowerSlide lowSlide;

        public LowerSlideCommands(LowerSlide lowSlide) {
                this.lowSlide = lowSlide;
        }

        // SLIDE POSITION COMMANDS
        private class SlidePositionCommand extends SlideCommand {
                private final double targetCm;

                public SlidePositionCommand(double cm) {
                        this.targetCm = cm;
                }

                @Override
                protected void setTargetPosition() {
                        lowSlide.setPositionCM(targetCm);
                }

                @Override
                protected void updatePID() {
                        lowSlide.updatePID();
                }

                @Override
                protected double getCurrentPosition() {
                        return lowSlide.getCurrentPosition();
                }

                @Override
                protected double getTargetPosition() {
                        return lowSlide.pidController.destination;
                }

                @Override
                protected String getTelemetryName() {
                        return "lowerslide";
                }
        }

        public Command setSlidePos(double cm) {
                return new SlidePositionCommand(cm);
        }

        public Command slidePos1() {
                return setSlidePos(50); // 50cm
        }

        public Command slidePos2() {
                return setSlidePos(0);
        }

        // PART1 AND PART2 COMMANDS
        private class Part1Command extends ServoCommand {
                public Part1Command(double pos) {
                        super("lowerslide/part1_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        lowSlide.setPart1Position(getTargetPosition());
                }
        }

        private class Part2Command extends ServoCommand {
                public Part2Command(double pos) {
                        super("lowerslide/part2_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        lowSlide.setPart2Position(getTargetPosition());
                }
        }

        public Command setPart1Pos(double pos) {
                return new Part1Command(pos);
        }

        public Command setPart2Pos(double pos) {
                return new Part2Command(pos);
        }

        public Command upPart1() {
                return setPart1Pos(LowerSlideVars.UP_BIG);
        }

        public Command upPart2() {
                return setPart2Pos(LowerSlideVars.UP_SMALL);
        }

        public Command grabPart1() {
                return setPart1Pos(LowerSlideVars.GRAB_BIG);
        }

        public Command grabPart2() {
                return setPart2Pos(LowerSlideVars.GRAB_SMALL);
        }

        public Command hoverPart1() {
                return setPart1Pos(LowerSlideVars.HOVER_BIG);
        }

        public Command hoverPart2() {
                return setPart2Pos(LowerSlideVars.HOVER_SMALL);
        }

        // SPINCLAW COMMANDS
        private class SpinClawCommand extends ServoCommand {
                public SpinClawCommand(double deg) {
                        super("lowerslide/spinclaw_deg", deg);
                }

                @Override
                protected void setServoPosition() {
                        lowSlide.spinclawSetPositionDeg(getTargetPosition());
                }
        }

        public Command setSpinClawDeg(double deg) {
                return new SpinClawCommand(deg);
        }

        public Command spinClaw0() {
                return setSpinClawDeg(0);
        }

        public Command spinClaw45() {
                return setSpinClawDeg(45);
        }

        public Command spinClaw90() {
                return setSpinClawDeg(90);
        }

        // CLAW COMMANDS
        private class ClawCommand extends ServoCommand {
                public ClawCommand(double pos) {
                        super("lowerslide/claw_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        if (getTargetPosition() == LowerSlideVars.CLAW_OPEN) {
                                lowSlide.openClaw();
                        } else {
                                lowSlide.closeClaw();
                        }
                }
        }

        public Command openClaw() {
                return new ClawCommand(LowerSlideVars.CLAW_OPEN);
        }

        public Command closeClaw() {
                return new ClawCommand(LowerSlideVars.CLAW_CLOSE);
        }

        // LEGACY COMBO COMMANDS
        public Command up() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                lowSlide.getCurrentPosition(), // Keep current position
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        public Command grab() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                lowSlide.getCurrentPosition(), // Keep current position
                                LowerSlideVars.GRAB_BIG,
                                LowerSlideVars.GRAB_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        public Command hover() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                lowSlide.getCurrentPosition(), // Keep current position
                                LowerSlideVars.HOVER_BIG,
                                LowerSlideVars.HOVER_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }
}
