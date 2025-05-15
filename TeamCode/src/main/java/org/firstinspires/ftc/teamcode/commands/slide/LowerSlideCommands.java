package org.firstinspires.ftc.teamcode.commands.slide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables.LowerSlideVars;
import org.firstinspires.ftc.teamcode.utils.control.ExpansionHub;

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
                        return lowSlide.pidfController.destination;
                }

                @Override
                protected String getTelemetryName() {
                        return "lowerslide";
                }
        }

        public Action setSlidePos(double cm) {
                return new SlidePositionCommand(cm);
        }

        public Action slidePos0() {
                return setSlidePos(LowerSlideVars.POS_0_CM);
        }
        public Action slidePos1() {
                return setSlidePos(LowerSlideVars.POS_1_CM);
        }
        public Action slidePos2() {
                return setSlidePos(LowerSlideVars.POS_2_CM);
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

        public Action setPart1Pos(double pos) {
                return new Part1Command(pos);
        }
        public Action zero(HardwareMap hardwareMap) {
                DcMotor slideMotor123;
                slideMotor123 = hardwareMap.get(DcMotor.class, ExpansionHub.motor(2));

                // Configure motor direction and mode
                slideMotor123.setDirection(DcMotor.Direction.REVERSE);
                slideMotor123.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor123.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                return new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                return false;
                        }
                };
        }

        public Action setPart2Pos(double pos) {
                return new Part2Command(pos);
        }

        public Action upPart1() {
                return setPart1Pos(LowerSlideVars.UP_BIG);
        }

        public Action upPart2() {
                return setPart2Pos(LowerSlideVars.UP_SMALL);
        }

        public Action grabPart1() {
                return setPart1Pos(LowerSlideVars.GRAB_BIG);
        }

        public Action grabPart2() {
                return setPart2Pos(LowerSlideVars.GRAB_SMALL);
        }

        public Action hoverPart1() {
                return setPart1Pos(LowerSlideVars.HOVER_BIG);
        }

        public Action hoverPart2() {
                return setPart2Pos(LowerSlideVars.HOVER_SMALL);
        }

        public Action hover() {
                return new ParallelAction(
                                hoverPart1(),
                                hoverPart2());
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

        public Action setSpinClawDeg(double deg) {
                return new SpinClawCommand(deg);
        }

        public Action spinClaw0() {
                return setSpinClawDeg(0);
        }

        public Action spinClaw45() {
                return setSpinClawDeg(45);
        }

        public Action spinClaw90() {
                return setSpinClawDeg(90);
        }

        public Action up() {
                return new ParallelAction(
                                upPart1(),
                                upPart2(),
                                setSpinClawDeg(LowerSlideVars.SPINCLAW_DEG));
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

        public Action openClaw() {
                return new ClawCommand(LowerSlideVars.CLAW_OPEN);
        }

        public Action closeClaw() {
                return new ClawCommand(LowerSlideVars.CLAW_CLOSE);
        }

}
