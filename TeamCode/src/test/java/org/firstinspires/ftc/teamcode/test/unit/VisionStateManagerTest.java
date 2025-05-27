package org.firstinspires.ftc.teamcode.test.unit;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.vision.VisionState;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Unit tests for VisionStateManager functionality.
 * Tests state transitions, detection handling, and calibration.
 */
public class VisionStateManagerTest {

        @Mock
        private LoopTimer loopTimer;

        private VisionStateManager stateManager;

        @Before
        public void setUp() {
                MockitoAnnotations.initMocks(this);
                stateManager = new VisionStateManager(loopTimer);
        }

        @Test
        public void testInitialState() {
                assertEquals("Should start in INITIALIZING state",
                                VisionState.INITIALIZING, stateManager.getCurrentState());
                assertFalse("Should not have detection initially",
                                stateManager.hasDetection());
                assertFalse("Should not be calibrated initially",
                                stateManager.isCalibrated());
        }

        @Test
        public void testCalibrationSequence() {
                // Start calibration
                assertFalse(stateManager.isCalibrated());
                assertTrue(stateManager.needsCalibration());

                // Complete calibration
                stateManager.setCalibrated(true);
                assertTrue(stateManager.isCalibrated());
                assertFalse(stateManager.needsCalibration());

                // Verify state transition
                assertEquals(VisionState.SEARCHING, stateManager.getCurrentState());
        }

        @Test
        public void testSingleDetectionTracking() {
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = new ArrayList<>();
                detections.add(new VisionStateManager.Detection(0.9, new Object()));

                // Update with high confidence detection
                stateManager.update(detections);
                assertTrue(stateManager.hasDetection());
                assertEquals(1, stateManager.getDetectionCount());
                assertEquals(0.9, stateManager.getConfidence(), 0.001);
                assertEquals(VisionState.TRACKING_SINGLE, stateManager.getCurrentState());
        }

        @Test
        public void testMultipleDetectionTracking() {
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = Arrays.asList(
                                new VisionStateManager.Detection(0.8, new Object()),
                                new VisionStateManager.Detection(0.7, new Object()));

                // Update with multiple detections
                stateManager.update(detections);
                assertTrue(stateManager.hasDetection());
                assertTrue(stateManager.hasMultipleDetections());
                assertEquals(2, stateManager.getDetectionCount());
                assertEquals(VisionState.TRACKING_MULTIPLE, stateManager.getCurrentState());
        }

        @Test
        public void testLowConfidenceHandling() {
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = Arrays.asList(
                                new VisionStateManager.Detection(0.4, new Object()));

                // Update with low confidence detection
                stateManager.update(detections);
                assertTrue(stateManager.hasDetection());
                assertTrue(stateManager.isDetectionDegrading());
                assertEquals(VisionState.TRACKING_WEAK, stateManager.getCurrentState());
        }

        @Test
        public void testLostTrackRecovery() {
                // First establish tracking
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = Arrays.asList(
                                new VisionStateManager.Detection(0.9, new Object()));
                stateManager.update(detections);
                assertEquals(VisionState.TRACKING_SINGLE, stateManager.getCurrentState());

                // Then lose detection
                stateManager.update(new ArrayList<>());
                assertEquals(VisionState.LOST_TRACK, stateManager.getCurrentState());
                assertFalse(stateManager.hasDetection());

                // Should transition to REACQUIRING after timeout
                for (int i = 0; i < 10; i++) { // Simulate multiple updates
                        stateManager.update(new ArrayList<>());
                }
                assertEquals(VisionState.REACQUIRING, stateManager.getCurrentState());
        }

        @Test
        public void testErrorHandling() {
                stateManager.setCalibrated(true);

                // Simulate pipeline error
                for (int i = 0; i < VisionStateManager.MAX_RETRIES + 1; i++) {
                        stateManager.update(null); // Trigger error
                }
                assertTrue(stateManager.getCurrentState().isErrorState());
                assertEquals(VisionState.ERROR_PIPELINE, stateManager.getCurrentState());
        }

        @Test
        public void testPerformanceMetrics() {
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = Arrays.asList(
                                new VisionStateManager.Detection(0.9, new Object()));

                // Verify timing sections are created
                stateManager.update(detections);
                verify(loopTimer, times(1)).startSection("vision_state");
                verify(loopTimer, times(1)).endSection("vision_state");
        }

        @Test
        public void testConfidenceAveraging() {
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = Arrays.asList(
                                new VisionStateManager.Detection(0.8, new Object()),
                                new VisionStateManager.Detection(0.6, new Object()));

                stateManager.update(detections);
                assertEquals(0.7, stateManager.getAverageConfidence(), 0.001);
        }

        @Test
        public void testReset() {
                // First establish some state
                stateManager.setCalibrated(true);
                List<VisionStateManager.Detection> detections = Arrays.asList(
                                new VisionStateManager.Detection(0.9, new Object()));
                stateManager.update(detections);

                // Then reset
                stateManager.reset();

                assertEquals(VisionState.INITIALIZING, stateManager.getCurrentState());
                assertFalse(stateManager.isCalibrated());
                assertFalse(stateManager.hasDetection());
                assertEquals(0, stateManager.getDetectionCount());
        }
}
