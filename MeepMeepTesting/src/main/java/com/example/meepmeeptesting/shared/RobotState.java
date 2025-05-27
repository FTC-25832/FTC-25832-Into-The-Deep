package com.example.meepmeeptesting.shared;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Enhanced robot state enum representing different positions, headings, and
 * state
 * relationships.
 * Each state includes position, heading, and phase information for autonomous
 * navigation.
 * States are grouped by autonomous phase and include validation/transition
 * rules.
 */
public enum RobotState {
        // Initialization/Start Phase
        START(40.1, 62.0, 270, Phase.INIT),
        PREPLACED(47.0, 46.0, -90, Phase.INIT),

        // Scoring Phase
        SCORE_APPROACH(52.0, 52.0, 180, Phase.SCORE),
        SCORE_POSITION(57.0, 57.0, 225, Phase.SCORE),
        SCORE_RETREAT(52.0, 52.0, 180, Phase.SCORE),

        // Pickup Phase
        PICKUP_1_APPROACH(47.1, 50.0, -90, Phase.PICKUP),
        PICKUP_1(47.1, 47.1, -90, Phase.PICKUP),
        PICKUP_2_APPROACH(58.0, 50.0, -90, Phase.PICKUP),
        PICKUP_2(58.0, 47.0, -90, Phase.PICKUP),
        PICKUP_3_APPROACH(50.0, 50.0, -45, Phase.PICKUP),
        PICKUP_3(47.1, 47.1, -45, Phase.PICKUP),

        // Transition Phase
        TANK_APPROACH(38.0, 5.0, 180, Phase.TRANSITION),
        TANK_FINAL(23.0, 5.0, 180, Phase.TRANSITION),

        // End Game/Hang Phase
        HANG_START(0.0, 30.0, -90, Phase.ENDGAME),
        HANG_APPROACH(-40.5, 40.5, -135, Phase.ENDGAME),
        HANG_ROTATION(-40.5, 40.5, 105, Phase.ENDGAME),
        HANG_FINAL(-40.5, 40.5, 105, Phase.ENDGAME);

        // Define autonomous phases
        public enum Phase {
                INIT,
                SCORE,
                PICKUP,
                TRANSITION,
                ENDGAME
        }

        private final double x;
        private final double y;
        private final double headingDegrees;
        private final Pose2d pose;
        private final Vector2d position;
        private final Phase phase;

        // Default tolerances
        private static final double DEFAULT_POSITION_TOLERANCE = 1.0; // inches
        private static final double DEFAULT_HEADING_TOLERANCE = 5.0; // degrees

        RobotState(double x, double y, double headingDegrees, Phase phase) {
                this.x = x;
                this.y = y;
                this.headingDegrees = headingDegrees;
                this.position = new Vector2d(x, y);
                this.pose = new Pose2d(x, y, Math.toRadians(headingDegrees));
                this.phase = phase;
        }

        // Getters
        public Vector2d getPosition() {
                return position;
        }

        public double getHeadingDegrees() {
                return headingDegrees;
        }

        public double getHeadingRadians() {
                return Math.toRadians(headingDegrees);
        }

        public Pose2d getPose() {
                return pose;
        }

        public double getX() {
                return x;
        }

        public double getY() {
                return y;
        }

        public Phase getPhase() {
                return phase;
        }

        /**
         * Check if robot is near a target state with default tolerances
         */
        public boolean isNear(RobotState other) {
                return isNear(other, DEFAULT_POSITION_TOLERANCE, DEFAULT_HEADING_TOLERANCE);
        }

        /**
         * Check if robot is near a target state with custom tolerances
         */
        public boolean isNear(RobotState other, double positionTolerance, double headingTolerance) {
                double dx = x - other.x;
                double dy = y - other.y;
                boolean positionMatch = Math.hypot(dx, dy) <= positionTolerance;
                double headingDiff = Math.abs(headingDegrees - other.headingDegrees) % 360;
                boolean headingMatch = Math.min(headingDiff, 360 - headingDiff) <= headingTolerance;
                return positionMatch && headingMatch;
        }

        /**
         * Check if state transition is valid
         */
        public boolean canTransitionTo(RobotState target) {
                // Prevent invalid phase transitions
                if (phase == Phase.INIT && target.phase != Phase.SCORE)
                        return false;
                if (phase == Phase.ENDGAME && target.phase != Phase.ENDGAME)
                        return false;

                // Ensure proper phase sequence
                switch (phase) {
                        case SCORE:
                                return target.phase == Phase.PICKUP || target.phase == Phase.ENDGAME;
                        case PICKUP:
                                return target.phase == Phase.SCORE || target.phase == Phase.TRANSITION;
                        case TRANSITION:
                                return target.phase == Phase.ENDGAME;
                        default:
                                return true;
                }
        }

        /**
         * Get approach state for current state if it exists
         */
        public RobotState getApproachState() {
                switch (this) {
                        case SCORE_POSITION:
                                return SCORE_APPROACH;
                        case PICKUP_1:
                                return PICKUP_1_APPROACH;
                        case PICKUP_2:
                                return PICKUP_2_APPROACH;
                        case PICKUP_3:
                                return PICKUP_3_APPROACH;
                        default:
                                return this;
                }
        }

        /**
         * Check if state requires specific subsystem actions
         */
        public boolean requiresSubsystemAction() {
                return phase == Phase.SCORE || phase == Phase.PICKUP || phase == Phase.ENDGAME;
        }

        /**
         * Get default approach speed multiplier for this state
         */
        public double getSpeedMultiplier() {
                switch (phase) {
                        case SCORE:
                        case PICKUP:
                                return 0.7; // Slower for precision
                        case ENDGAME:
                                return 0.5; // Very slow for hang
                        default:
                                return 1.0;
                }
        }
}
