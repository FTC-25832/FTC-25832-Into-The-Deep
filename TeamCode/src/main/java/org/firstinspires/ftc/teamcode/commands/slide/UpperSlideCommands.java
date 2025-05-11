package org.firstinspires.ftc.teamcode.commands.slide;

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

        /**
         * Create command to move to position 0 (down)
         */
        public Command pos0() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_0_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        /**
         * Create command to move to position 1
         */
        public Command pos1() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_1_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        /**
         * Create command to move to position 2
         */
        public Command pos2() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_2_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        /**
         * Create command to move to position 3
         */
        public Command pos3() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                UpperSlideVars.POS_3_CM,
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        /**
         * Create command to move to transfer position
         */
        public Command transfer() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                0, // Slide stays at current position
                                UpperSlideVars.BEHIND_ARM_POS,
                                UpperSlideVars.BEHIND_SWING_POS);
        }

        /**
         * Create command to move to front position
         */
        public Command front() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                0, // Slide stays at current position
                                UpperSlideVars.FRONT_ARM_POS,
                                UpperSlideVars.FRONT_SWING_POS);
        }

        /**
         * Create command to move to off-wall position
         */
        public Command offwall() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                0, // Slide stays at current position
                                UpperSlideVars.OFFWALL_FRONT_ARM_POS,
                                UpperSlideVars.OFFWALL_FRONT_SWING_POS);
        }

        /**
         * Create command to move to score spec position
         */
        public Command scorespec() {
                return new UpperSlidePositionCommand(
                                upSlide,
                                0, // Slide stays at current position
                                UpperSlideVars.SCORESPEC_FRONT_ARM_POS,
                                UpperSlideVars.SCORESPEC_FRONT_SWING_POS);
        }
}
