package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Enum representing different robot states with their associated positions and
 * headings. Mirrors the shared implementation for consistency.
 */
public enum RobotState {
        // Starting positions
        START(40.1, 62.0, 270),
        PREPLACED(47.0, 46.0, -90),

        // Pickup positions
        PICKUP_1(47.1, 47.1, -90),
        PICKUP_2(58.0, 47.0, -90),
        PICKUP_3(47.1, 47.1, -45),

        // Scoring positions
        SCORE(57.0, 57.0, 225),

        // Transition positions
        TANK_APPROACH(38.0, 5.0, 180),
        TANK_FINAL(23.0, 5.0, 180),

        // Hang positions
        HANG_START(0.0, 30.0, -90),
        HANG_APPROACH(-40.5, 40.5, -135),
        HANG_ROTATION(-40.5, 40.5, 105);

        private final double x;
        private final double y;
        private final double headingDegrees;
        private final Pose2d pose;
        private final Vector2d position;

        RobotState(double x, double y, double headingDegrees) {
                this.x = x;
                this.y = y;
                this.headingDegrees = headingDegrees;
                this.position = new Vector2d(x, y);
                this.pose = new Pose2d(x, y, Math.toRadians(headingDegrees));
        }

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

        public boolean isNear(RobotState other, double positionTolerance, double headingTolerance) {
                double dx = x - other.x;
                double dy = y - other.y;
                boolean positionMatch = Math.hypot(dx, dy) <= positionTolerance;
                double headingDiff = Math.abs(headingDegrees - other.headingDegrees) % 360;
                boolean headingMatch = Math.min(headingDiff, 360 - headingDiff) <= headingTolerance;
                return positionMatch && headingMatch;
        }

        /**
         * Check if the robot's current state is valid for position tracking
         * 
         * @return true if the state is valid for tracking
         */
        public boolean isValidTrackingState() {
                // Avoid tracking during high-speed or unstable states
                return this != TANK_APPROACH && this != TANK_FINAL;
        }

        /**
         * Get the next expected state in a typical autonomous sequence
         * 
         * @return The next logical state, or null if this is a terminal state
         */
        public RobotState getNextState() {
                switch (this) {
                        case START:
                                return PREPLACED;
                        case PREPLACED:
                                return PICKUP_1;
                        case PICKUP_1:
                                return SCORE;
                        case SCORE:
                                return PICKUP_2;
                        case PICKUP_2:
                                return SCORE;
                        case PICKUP_3:
                                return SCORE;
                        default:
                                return null;
                }
        }
}
