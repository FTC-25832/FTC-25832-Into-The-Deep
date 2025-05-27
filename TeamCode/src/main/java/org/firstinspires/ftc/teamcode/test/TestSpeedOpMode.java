package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.utils.control.ControlHub;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;

/**
 * Enhanced test OpMode to validate bulk reads and performance monitoring.
 * Tests hardware access optimization using BulkReadManager.
 * Measures and compares:
 * - Regular vs bulk read performance
 * - Caching effectiveness
 * - Loop time statistics
 * - Individual component read times
 * Supports testing with motors and IMUs
 */
@TeleOp(name = "Test: Hardware Performance", group = "Test")
public class TestSpeedOpMode extends BaseOpMode {
        private DcMotorEx[] motors = new DcMotorEx[4];
        private IMU imu;
        private BulkReadManager bulkReader;
        private final ElapsedTime timer = new ElapsedTime();
        private int testPhase = 0;
        private static final int TEST_CYCLES = 2000; // More cycles for better statistics
        private int cycleCount = 0;

        // Performance metrics
        private double regularReadTime = 0;
        private double bulkReadTime = 0;
        private double maxLoopTime = 0;
        private double minLoopTime = Double.MAX_VALUE;
        private int loopsOver10ms = 0;
        private int loopsOver20ms = 0;
        private double avgImuReadTime = 0;
        private double avgMotorReadTime = 0;
        private int communicationErrors = 0;

        @Override
        protected void initialize() {
                // Initialize hardware
                for (int i = 0; i < 4; i++) {
                        motors[i] = hardwareMap.get(DcMotorEx.class, ControlHub.motor(i));
                }
                imu = hardwareMap.get(IMU.class, "imu");

                // Initialize BulkReadManager
                bulkReader = BulkReadManager.getInstance(hardwareMap);

                // Register hardware for monitoring
                for (DcMotorEx motor : motors) {
                        bulkReader.addMotor(motor);
                }
                bulkReader.addIMU(imu);

                // Configure for manual bulk caching
                for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                        module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                }

                telemetry.setMsTransmissionInterval(50);
                telemetry.addData("Status", "Initialized");
                telemetry.addData("Instructions", "Press A: Regular reads test");
                telemetry.addData("", "Press B: Bulk reads test");
                telemetry.addData("", "Press X: Reset stats");
                telemetry.addData("", "Press Y: Toggle detailed metrics");
                telemetry.update();
        }

        @Override
        protected void update() {
                if (bulkReader != null) {
                        bulkReader.getLoopTimer().startLoop();
                }

                try {
                        switch (testPhase) {
                                case 0: // Waiting to start
                                        handleInputs();
                                        break;

                                case 1: // Regular reads test
                                        if (cycleCount < TEST_CYCLES) {
                                                performRegularReads();
                                        } else {
                                                regularReadTime = timer.milliseconds() / TEST_CYCLES;
                                                testPhase = 0;
                                                cycleCount = 0;
                                        }
                                        break;

                                case 2: // Bulk reads test
                                        if (cycleCount < TEST_CYCLES) {
                                                performBulkReads();
                                        } else {
                                                bulkReadTime = timer.milliseconds() / TEST_CYCLES;
                                                testPhase = 0;
                                                cycleCount = 0;
                                        }
                                        break;
                        }
                } catch (Exception e) {
                        communicationErrors++;
                        telemetry.addData("Error", e.getMessage());
                }

                if (bulkReader != null) {
                        bulkReader.getLoopTimer().endLoop();
                        updateLoopStatistics(bulkReader.getLoopTimer());
                }
        }

        private void handleInputs() {
                if (gamepad1.a) {
                        startTest(1, "Regular Reads");
                } else if (gamepad1.b) {
                        startTest(2, "Bulk Reads");
                } else if (gamepad1.x) {
                        resetStatistics();
                }
        }

        private void startTest(int phase, String description) {
                resetStatistics();
                testPhase = phase;
                cycleCount = 0;
                timer.reset();
                telemetry.addData("Status", "Running " + description + " test...");
                telemetry.update();
        }

        private void resetStatistics() {
                regularReadTime = 0;
                bulkReadTime = 0;
                maxLoopTime = 0;
                minLoopTime = Double.MAX_VALUE;
                loopsOver10ms = 0;
                loopsOver20ms = 0;
                avgImuReadTime = 0;
                avgMotorReadTime = 0;
                communicationErrors = 0;
                if (bulkReader != null) {
                        bulkReader.getLoopTimer().reset();
                }
        }

        private void performRegularReads() {
                timer.reset();
                double startTime = timer.milliseconds();

                // Individual motor reads
                for (DcMotorEx motor : motors) {
                        motor.getCurrentPosition();
                        motor.getVelocity();
                        motor.getCurrent(CurrentUnit.AMPS);
                }
                avgMotorReadTime = (avgMotorReadTime * cycleCount + (timer.milliseconds() - startTime))
                                / (cycleCount + 1);

                // IMU reads
                startTime = timer.milliseconds();
                imu.getRobotYawPitchRollAngles();
                avgImuReadTime = (avgImuReadTime * cycleCount + (timer.milliseconds() - startTime)) / (cycleCount + 1);

                updateLoopStats(timer.milliseconds());
                cycleCount++;
        }

        private void performBulkReads() {
                timer.reset();

                // Clear cache and perform bulk reads
                bulkReader.clearCache();

                double startTime = timer.milliseconds();
                for (DcMotorEx motor : motors) {
                        bulkReader.getMotorPosition(motor);
                        bulkReader.getMotorVelocity(motor);
                        bulkReader.getMotorCurrent(motor);
                }
                avgMotorReadTime = (avgMotorReadTime * cycleCount + (timer.milliseconds() - startTime))
                                / (cycleCount + 1);

                // IMU data through bulk reader
                startTime = timer.milliseconds();
                bulkReader.getIMUData(imu);
                avgImuReadTime = (avgImuReadTime * cycleCount + (timer.milliseconds() - startTime)) / (cycleCount + 1);

                updateLoopStats(timer.milliseconds());
                cycleCount++;
        }

        private void updateLoopStats(double loopTime) {
                maxLoopTime = Math.max(maxLoopTime, loopTime);
                minLoopTime = Math.min(minLoopTime, loopTime);
                if (loopTime > 20) {
                        loopsOver20ms++;
                } else if (loopTime > 10) {
                        loopsOver10ms++;
                }
        }

        private void updateLoopStatistics(LoopTimer loopTimer) {
                if (loopTimer != null) {
                        double loopTime = loopTimer.getStats("FullLoop").getAverage();
                        updateLoopStats(loopTime);
                }
        }

        @Override
        protected void addCustomTelemetry() {
                telemetry.addLine("=== Test Status ===");
                telemetry.addData("Phase", getTestPhaseName());
                telemetry.addData("Cycles", "%d/%d", cycleCount, TEST_CYCLES);
                telemetry.addData("Errors", communicationErrors);

                telemetry.addLine("\n=== Performance Metrics ===");
                if (regularReadTime > 0) {
                        telemetry.addData("Regular Read Time", "%.2f ms/cycle", regularReadTime);
                        telemetry.addData("- Motors", "%.2f ms", avgMotorReadTime);
                        telemetry.addData("- IMU", "%.2f ms", avgImuReadTime);
                }

                if (bulkReadTime > 0) {
                        telemetry.addData("Bulk Read Time", "%.2f ms/cycle", bulkReadTime);
                        if (regularReadTime > 0) {
                                double improvement = ((regularReadTime - bulkReadTime) / regularReadTime) * 100;
                                telemetry.addData("Improvement", "%.1f%%", improvement);
                        }
                }

                telemetry.addLine("\n=== Loop Statistics ===");
                telemetry.addData("Loop Time", "Min: %.2f ms, Max: %.2f ms", minLoopTime, maxLoopTime);
                telemetry.addData("Slow Loops", ">10ms: %d, >20ms: %d", loopsOver10ms, loopsOver20ms);

                if (bulkReader != null) {
                        telemetry.addLine("\n=== Bulk Read Details ===");
                        LoopTimer loopTimer = bulkReader.getLoopTimer();
                        for (String section : new String[] { "BulkRead", "ClearCache" }) {
                                reportTimingStats(telemetry, loopTimer, section);
                        }
                }
        }

        private void reportTimingStats(Telemetry telemetry, LoopTimer loopTimer, String section) {
                if (loopTimer != null) {
                        TimingStats stats = loopTimer.getStats(section);
                        if (stats != null) {
                                telemetry.addData(section,
                                                "Avg: %.2fms, Min: %.2fms, Max: %.2fms",
                                                stats.getAverage(),
                                                stats.getMin(),
                                                stats.getMax());
                        }
                }
        }

        private String getTestPhaseName() {
                switch (testPhase) {
                        case 0:
                                return "Waiting";
                        case 1:
                                return "Regular Reads";
                        case 2:
                                return "Bulk Reads";
                        default:
                                return "Unknown";
                }
        }
}
