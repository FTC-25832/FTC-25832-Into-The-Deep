package org.firstinspires.ftc.teamcode.commands.slide;

import android.graphics.Bitmap;
import android.telecom.Conference;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.commands.base.WaitCommand;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;
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

        public Action setSlidePos(double cm) {
                return new SlidePositionCommand(cm);
        }
        public Action addSlideTick(int direction){
                return telemetryPacket -> {
                        upSlide.setTickOffset(upSlide.tickOffset + UpperSlideVars.SET_TICK_SPEED * direction);
                        telemetryPacket.put("upperslide/tick", upSlide.tickOffset);
                        return false;
                };
        }

        public Action slidePos0() {
                return new SequentialAction(
                                setSlidePos(UpperSlideVars.POS_PRE_0_CM),
                                new WaitCommand(0.1).toAction(),
                                setSlidePos(UpperSlideVars.POS_0_CM));
        }

        public Action slidePos1() {
                return setSlidePos(UpperSlideVars.POS_1_CM);
        }

        public Action slidePos2() {
                return setSlidePos(UpperSlideVars.POS_2_CM);
        }

        public Action slidePos3() {
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

        public Action setArmPos(double pos) {
                return new ArmCommand(pos);
        }

        public Action frontArm() {
                return setArmPos(UpperSlideVars.FRONT_ARM_POS);
        }

        public Action behindArm() {
                return setArmPos(UpperSlideVars.BEHIND_ARM_POS);
        }

        public Action offwallArm() {
                return setArmPos(UpperSlideVars.OFFWALL_FRONT_ARM_POS);
        }

        public Action scorespecArm() {
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

        public Action setSwingPos(double pos) {
                return new SwingCommand(pos);
        }

        public Action frontSwing() {
                return setSwingPos(UpperSlideVars.FRONT_SWING_POS);
        }

        public Action behindSwing() {
                return setSwingPos(UpperSlideVars.BEHIND_SWING_POS);
        }

        public Action offwallSwing() {
                return setSwingPos(UpperSlideVars.OFFWALL_FRONT_SWING_POS);
        }

        public Action scorespecSwing() {
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

        private class ExtendoClawCommand extends ServoCommand {
                public ExtendoClawCommand(double pos) {
                        super("upperslide/extendo_claw_target", pos);
                }

                @Override
                protected void setServoPosition() {
                        if (getTargetPosition() == UpperSlideVars.EXTENDO_OPEN) {
                                upSlide.openExtendoClaw();
                        } else {
                                upSlide.closeExtendoClaw();
                        }
                }
        }

        public Action openClaw() {
                return new ClawCommand(UpperSlideVars.CLAW_OPEN);
        }

        public Action closeClaw() {
                return new ClawCommand(UpperSlideVars.CLAW_CLOSE);
        }

        public Action openExtendoClaw() { return new ExtendoClawCommand(UpperSlideVars.EXTENDO_OPEN); }

        public Action closeExtendoClaw() { return new ExtendoClawCommand(UpperSlideVars.EXTENDO_CLOSE); }



        public Action transfer() {
                return new SequentialAction(
                                setArmPos(UpperSlideVars.BEHIND_ARM_POS),
                                setSwingPos(UpperSlideVars.BEHIND_SWING_POS));
        }

        public Action front() {
                return new SequentialAction(
                                setArmPos(UpperSlideVars.FRONT_ARM_POS),
                                setSwingPos(UpperSlideVars.FRONT_SWING_POS));
        }

        public Action offwall() {
                return new SequentialAction(
                        setArmPos(UpperSlideVars.OFFWALL_FRONT_ARM_POS),
                        setSwingPos(UpperSlideVars.OFFWALL_FRONT_SWING_POS));
        }

        public Action scorespec() {
                return new SequentialAction(
                                setArmPos(UpperSlideVars.SCORESPEC_FRONT_ARM_POS),
                                setSwingPos(UpperSlideVars.SCORESPEC_FRONT_SWING_POS));
        }

        public Action inter() {
                return new SequentialAction(
                        setArmPos(UpperSlideVars.INTER_ARM_POS),
                        setSwingPos(UpperSlideVars.INTER_SWING_POS));
        }
}
