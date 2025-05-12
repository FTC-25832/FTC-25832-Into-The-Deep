package org.firstinspires.ftc.teamcode.commands.slide;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables.UpperSlideVars;

/**
 * Factory class for creating upper slide commands
 */
public class UpperSlideCommands {
        private final UpperSlide upSlide;

        public UpperSlideCommands(UpperSlide upSlide) {
                this.upSlide = upSlide;
        }

        // SLIDE POSITION COMMANDS
        private class SlidePositionCommand extends SlideCommand {
                private final double targetCm;

                public SlidePositionCommand(double cm) {
                        this.targetCm = cm;
                }

                @Override
                protected void setTargetPosition() {
                        upSlide.setPositionCM(targetCm);
                }

                @Override
                protected void updatePID() {
                        upSlide.updatePID();
                }

                @Override
                protected double getCurrentPosition() {
                        return upSlide.getCurrentPosition();
                }

                @Override
                protected double getTargetPosition() {
                        return upSlide.pidfController.destination;
                }

                @Override
                protected String getTelemetryName() {
                        return "upperslide";
                }
        }

        public Command setSlidePos(double cm) {
                return new SlidePositionCommand(cm);
        }

        public Command slidePos0() {
                return setSlidePos(UpperSlideVars.POS_0_CM);
        }

        public Command slidePos1() {
                return setSlidePos(UpperSlideVars.POS_1_CM);
        }

        public Command slidePos2() {
                return setSlidePos(UpperSlideVars.POS_2_CM);
        }

        public Command slidePos3() {
                return setSlidePos(UpperSlideVars.POS_3_CM);
        }

        // ARM COMMANDS
        private class ArmCommand extends ServoCommand {
                public ArmCommand(double pos) {
                        super("upperslide/arm_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        upSlide.setArmPosition(getTargetPosition());
                }
        }

        public Command setArmPos(double pos) {
                return new ArmCommand(pos);
        }

        public Command frontArm() {
                return setArmPos(UpperSlideVars.FRONT_ARM_POS);
        }

        public Command behindArm() {
                return setArmPos(UpperSlideVars.BEHIND_ARM_POS);
        }

        public Command offwallArm() {
                return setArmPos(UpperSlideVars.OFFWALL_FRONT_ARM_POS);
        }

        public Command scorespecArm() {
                return setArmPos(UpperSlideVars.SCORESPEC_FRONT_ARM_POS);
        }

        // SWING COMMANDS
        private class SwingCommand extends ServoCommand {
                public SwingCommand(double pos) {
                        super("upperslide/swing_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        upSlide.setSwingPosition(getTargetPosition());
                }
        }

        public Command setSwingPos(double pos) {
                return new SwingCommand(pos);
        }

        public Command frontSwing() {
                return setSwingPos(UpperSlideVars.FRONT_SWING_POS);
        }

        public Command behindSwing() {
                return setSwingPos(UpperSlideVars.BEHIND_SWING_POS);
        }

        public Command offwallSwing() {
                return setSwingPos(UpperSlideVars.OFFWALL_FRONT_SWING_POS);
        }

        public Command scorespecSwing() {
                return setSwingPos(UpperSlideVars.SCORESPEC_FRONT_SWING_POS);
        }

        // CLAW COMMANDS
        private class ClawCommand extends ServoCommand {
                public ClawCommand(double pos) {
                        super("upperslide/claw_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        if (getTargetPosition() == UpperSlideVars.CLAW_OPEN) {
                                upSlide.openClaw();
                        } else {
                                upSlide.closeClaw();
                        }
                }
        }

        public Command openClaw() {
                return new ClawCommand(UpperSlideVars.CLAW_OPEN);
        }

        public Command closeClaw() {
                return new ClawCommand(UpperSlideVars.CLAW_CLOSE);
        }

        // LEGACY COMBO COMMANDS
        public Command pos0() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_0_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        public Command pos1() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_1_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        public Command pos2() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_2_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        public Command pos3() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_3_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        public Command transfer() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                upSlide.getCurrentPosition(), // Keep current position
                                UpperSlideVars.BEHIND_ARM_POS,
                                UpperSlideVars.BEHIND_SWING_POS);
        }

        public Command front() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                upSlide.getCurrentPosition(), // Keep current position
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        public Command offwall() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                upSlide.getCurrentPosition(), // Keep current position
                                UpperSlideVars.OFFWALL_FRONT_ARM_POS,
                                UpperSlideVars.OFFWALL_FRONT_SWING_POS);
        }

        public Command scorespec() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                upSlide.getCurrentPosition(), // Keep current position
                                UpperSlideVars.SCORESPEC_FRONT_ARM_POS,
                                UpperSlideVars.SCORESPEC_FRONT_SWING_POS);
        }
}
