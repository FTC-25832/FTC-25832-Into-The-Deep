package org.firstinspires.ftc.teamcode.commands.slide;

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

        /**
         * Create command to move slide to up position
         */
        public Command up() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0, // Keep current position
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        /**
         * Create command to move slide to grab position
         */
        public Command grab() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0, // Keep current position
                                LowerSlideVars.GRAB_BIG,
                                LowerSlideVars.GRAB_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        /**
         * Create command to move slide to hover position
         */
        public Command hover() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0, // Keep current position
                                LowerSlideVars.HOVER_BIG,
                                LowerSlideVars.HOVER_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        /**
         * Create command to move to slide position 1
         */
        public Command slidePos1() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                50, // 50cm
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        /**
         * Create command to move to slide position 2 (home)
         */
        public Command slidePos2() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0,
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                LowerSlideVars.SPINCLAW_DEG);
        }

        /**
         * Create command to set spinclaw to 45 degrees
         */
        public Command spinClaw45() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0, // Keep current position
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                45);
        }

        /**
         * Create command to set spinclaw to 0 degrees
         */
        public Command spinClaw0() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0, // Keep current position
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                0);
        }

        /**
         * Create command to set spinclaw to 90 degrees
         */
        public Command spinClaw90() {
                return new LowerSlidePositionCommand(
                                lowSlide,
                                0, // Keep current position
                                LowerSlideVars.UP_BIG,
                                LowerSlideVars.UP_SMALL,
                                90);
        }
}
