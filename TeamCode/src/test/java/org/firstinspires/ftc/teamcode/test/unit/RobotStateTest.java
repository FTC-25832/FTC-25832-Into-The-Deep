package org.firstinspires.ftc.teamcode.test.unit;

import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.Test;

import com.example.meepmeeptesting.shared.RobotState;
import com.example.meepmeeptesting.shared.RobotState.Phase;

/**
 * Unit tests for RobotState enum functionality.
 * Tests state transitions, validations, and calculations.
 */
public class RobotStateTest {

        @Test
        public void testStateTransitions() {
                // Test valid transitions
                assertTrue("Should allow START to SCORE transition",
                                RobotState.START.canTransitionTo(RobotState.SCORE_POSITION));
                assertTrue("Should allow SCORE to PICKUP transition",
                                RobotState.SCORE_POSITION.canTransitionTo(RobotState.PICKUP_1));
                assertTrue("Should allow PICKUP to SCORE transition",
                                RobotState.PICKUP_1.canTransitionTo(RobotState.SCORE_POSITION));

                // Test invalid transitions
                assertFalse("Should not allow START to PICKUP direct transition",
                                RobotState.START.canTransitionTo(RobotState.PICKUP_1));
                assertFalse("Should not allow SCORE to HANG direct transition",
                                RobotState.SCORE_POSITION.canTransitionTo(RobotState.HANG_FINAL));
        }

        @Test
        public void testPhaseProgression() {
                assertEquals("START should be in INIT phase",
                                Phase.INIT, RobotState.START.getPhase());
                assertEquals("SCORE should be in SCORE phase",
                                Phase.SCORE, RobotState.SCORE_POSITION.getPhase());
                assertEquals("PICKUP should be in PICKUP phase",
                                Phase.PICKUP, RobotState.PICKUP_1.getPhase());
                assertEquals("HANG should be in ENDGAME phase",
                                Phase.ENDGAME, RobotState.HANG_FINAL.getPhase());
        }

        @Test
        public void testPositionCalculations() {
                // Test position calculations within acceptable tolerance
                double tolerance = 0.01;

                assertEquals("X coordinate should match",
                                40.1, RobotState.START.getX(), tolerance);
                assertEquals("Y coordinate should match",
                                62.0, RobotState.START.getY(), tolerance);
                assertEquals("Heading should match",
                                270.0, RobotState.START.getHeadingDegrees(), tolerance);
        }

        @Test
        public void testApproachStates() {
                // Test approach state relationships
                assertEquals("SCORE approach should match",
                                RobotState.SCORE_APPROACH, RobotState.SCORE_POSITION.getApproachState());
                assertEquals("PICKUP_1 approach should match",
                                RobotState.PICKUP_1_APPROACH, RobotState.PICKUP_1.getApproachState());
        }

        @Test
        public void testSpeedMultipliers() {
                // Test speed multipliers for different phases
                assertEquals("Score phase should have reduced speed",
                                0.7, RobotState.SCORE_POSITION.getSpeedMultiplier(), 0.01);
                assertEquals("Endgame phase should have minimum speed",
                                0.5, RobotState.HANG_FINAL.getSpeedMultiplier(), 0.01);
                assertEquals("Init phase should have full speed",
                                1.0, RobotState.START.getSpeedMultiplier(), 0.01);
        }

        @Test
        public void testStateProximity() {
                // Test isNear calculations
                assertTrue("Same state should be near itself",
                                RobotState.START.isNear(RobotState.START));

                // Test with custom tolerances
                assertTrue("States should be near with loose tolerances",
                                RobotState.SCORE_APPROACH.isNear(RobotState.SCORE_POSITION, 10.0, 45.0));
                assertFalse("States should not be near with tight tolerances",
                                RobotState.SCORE_APPROACH.isNear(RobotState.SCORE_POSITION, 0.1, 1.0));
        }

        @Test
        public void testPhaseValidation() {
                for (RobotState state : RobotState.values()) {
                        assertNotNull("Phase should not be null", state.getPhase());
                        assertTrue("Phase should be valid type",
                                        state.getPhase() instanceof Phase);
                }
        }
}
