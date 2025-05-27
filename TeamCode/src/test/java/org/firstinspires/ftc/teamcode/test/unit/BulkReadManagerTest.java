package org.firstinspires.ftc.teamcode.test.unit;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;

import java.util.Arrays;
import java.util.List;

/**
 * Unit tests for BulkReadManager functionality.
 * Tests caching, performance monitoring, and error handling.
 */
public class BulkReadManagerTest {

        @Mock
        private HardwareMap hardwareMap;

        @Mock
        private DcMotorEx motor1, motor2;

        @Mock
        private IMU imu;

        @Mock
        private LynxModule lynxModule;

        @Mock
        private LoopTimer loopTimer;

        private BulkReadManager bulkReader;

        @Before
        public void setUp() {
                MockitoAnnotations.initMocks(this);

                // Setup mock hardware map
                when(hardwareMap.getAll(LynxModule.class))
                                .thenReturn(Arrays.asList(lynxModule));

                // Configure mock LynxModule
                when(lynxModule.getBulkCachingMode())
                                .thenReturn(LynxModule.BulkCachingMode.MANUAL);

                bulkReader = BulkReadManager.getInstance(hardwareMap);
                bulkReader.setLoopTimer(loopTimer);
        }

        @Test
        public void testComponentRegistration() {
                // Test motor registration
                bulkReader.addMotor(motor1);
                bulkReader.addMotor(motor2);
                assertTrue("Motor should be registered",
                                bulkReader.isMotorRegistered(motor1));
                assertTrue("Motor should be registered",
                                bulkReader.isMotorRegistered(motor2));

                // Test IMU registration
                bulkReader.addIMU(imu);
                assertTrue("IMU should be registered",
                                bulkReader.isIMURegistered(imu));
        }

        @Test
        public void testCacheBehavior() {
                bulkReader.addMotor(motor1);

                // First read should trigger bulk read
                bulkReader.getMotorPosition(motor1);
                verify(lynxModule, times(1)).clearBulkCache();
                verify(lynxModule, times(1)).getBulkData();

                // Second read should use cache
                bulkReader.getMotorPosition(motor1);
                verify(lynxModule, times(1)).getBulkData(); // Should not increment
        }

        @Test
        public void testCacheClearing() {
                bulkReader.addMotor(motor1);

                // Initial read
                bulkReader.getMotorPosition(motor1);
                verify(lynxModule, times(1)).getBulkData();

                // Clear cache
                bulkReader.clearCache();

                // Next read should trigger new bulk read
                bulkReader.getMotorPosition(motor1);
                verify(lynxModule, times(2)).getBulkData();
        }

        @Test
        public void testPerformanceTracking() {
                bulkReader.addMotor(motor1);

                // Verify loop timing sections are created
                bulkReader.getMotorPosition(motor1);
                verify(loopTimer, times(1)).startSection("bulk_read");
                verify(loopTimer, times(1)).endSection("bulk_read");
        }

        @Test
        public void testErrorHandling() {
                bulkReader.addMotor(motor1);

                // Simulate bulk read failure
                when(lynxModule.getBulkData()).thenThrow(new RuntimeException("Bulk read failed"));

                try {
                        bulkReader.getMotorPosition(motor1);
                        fail("Should throw exception on bulk read failure");
                } catch (RuntimeException e) {
                        assertEquals("Bulk read failed", e.getMessage());
                }
        }

        @Test
        public void testUnregisteredComponentAccess() {
                // Attempt to read unregistered motor
                assertThrows(IllegalStateException.class, () -> {
                        bulkReader.getMotorPosition(motor1);
                });

                // Attempt to read unregistered IMU
                assertThrows(IllegalStateException.class, () -> {
                        bulkReader.getIMUData(imu);
                });
        }

        @Test
        public void testMultipleReads() {
                bulkReader.addMotor(motor1);
                bulkReader.addMotor(motor2);

                // Multiple reads should use same cache
                bulkReader.getMotorPosition(motor1);
                bulkReader.getMotorPosition(motor2);
                bulkReader.getMotorVelocity(motor1);
                bulkReader.getMotorCurrent(motor2);

                // Should only do one bulk read
                verify(lynxModule, times(1)).getBulkData();
        }

        @Test
        public void testResetBehavior() {
                bulkReader.addMotor(motor1);
                bulkReader.addIMU(imu);

                bulkReader.reset();

                assertFalse("Motor should be unregistered",
                                bulkReader.isMotorRegistered(motor1));
                assertFalse("IMU should be unregistered",
                                bulkReader.isIMURegistered(imu));
        }
}
