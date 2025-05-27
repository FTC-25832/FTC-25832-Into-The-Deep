package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxNackException;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Enhanced bulk read manager with comprehensive hardware support and optimized
 * caching.
 * Implements true bulk reads from expansion hubs and detailed performance
 * monitoring.
 * Supports motors, IMUs, voltage sensors and other hardware components.
 */
public class BulkReadManager {
        private final List<LynxModule> allHubs;
        private final HardwareMap hardwareMap;
        private static BulkReadManager instance;
        private final Map<String, MotorData> motorDataCache;
        private final Map<String, IMUData> imuDataCache;
        private final Map<LynxModule, LynxModuleData> moduleDataCache;
        private final LoopTimer loopTimer;
        private boolean cacheValid = false;
        private long lastCacheTime = 0;
        private static final long CACHE_TIMEOUT_NS = 10_000_000; // 10ms cache timeout

        private static class MotorData {
                int position;
                double velocity;
                double current;
                long timestamp;

                MotorData() {
                        this.position = 0;
                        this.velocity = 0.0;
                        this.current = 0.0;
                        this.timestamp = 0;
                }
        }

        private static class IMUData {
                double heading;
                double roll;
                double pitch;
                long timestamp;

                IMUData() {
                        this.heading = 0.0;
                        this.roll = 0.0;
                        this.pitch = 0.0;
                        this.timestamp = 0;
                }
        }

        private static class LynxModuleData {
                double voltage;
                Map<Integer, Integer> encoderPositions;
                Map<Integer, Double> motorVelocities;
                Map<Integer, Double> motorCurrents;
                long timestamp;

                LynxModuleData() {
                        this.voltage = 0.0;
                        this.encoderPositions = new ConcurrentHashMap<>();
                        this.motorVelocities = new ConcurrentHashMap<>();
                        this.motorCurrents = new ConcurrentHashMap<>();
                        this.timestamp = 0;
                }
        }

        private BulkReadManager(HardwareMap hardwareMap) {
                this.hardwareMap = hardwareMap;
                motorDataCache = new ConcurrentHashMap<>();
                imuDataCache = new ConcurrentHashMap<>();
                moduleDataCache = new ConcurrentHashMap<>();
                loopTimer = new LoopTimer();
                allHubs = hardwareMap.getAll(LynxModule.class);

                // Configure all hubs for manual bulk caching mode
                for (LynxModule module : allHubs) {
                        module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                        moduleDataCache.put(module, new LynxModuleData());
                }
        }

        /**
         * Get cached motor data, performing bulk read if necessary
         */
        private MotorData getMotorData(DcMotorEx motor) {
                String motorName = motor.getDeviceName();
                if (!cacheValid || isCacheExpired()) {
                        refreshCache();
                }
                return motorDataCache.computeIfAbsent(motorName, k -> new MotorData());
        }

        /**
         * Get the current position of a motor
         */
        public int getMotorPosition(DcMotorEx motor) {
                return getMotorData(motor).position;
        }

        /**
         * Get the current velocity of a motor
         */
        public double getMotorVelocity(DcMotorEx motor) {
                return getMotorData(motor).velocity;
        }

        /**
         * Get the current draw of a motor
         */
        public double getMotorCurrent(DcMotorEx motor) {
                return getMotorData(motor).current;
        }

        /**
         * Get cached IMU data
         */
        public IMUData getIMUData(IMU imu) {
                String imuName = imu.getDeviceName();
                if (!cacheValid || isCacheExpired()) {
                        refreshCache();
                }
                return imuDataCache.computeIfAbsent(imuName, k -> new IMUData());
        }

        /**
         * Get voltage from a specific module
         */
        public double getVoltage(LynxModule module) {
                if (!cacheValid || isCacheExpired()) {
                        refreshCache();
                }
                LynxModuleData data = moduleDataCache.get(module);
                return data != null ? data.voltage : 0.0;
        }

        /**
         * Check if cache has expired
         */
        private boolean isCacheExpired() {
                return System.nanoTime() - lastCacheTime > CACHE_TIMEOUT_NS;
        }

        /**
         * Refresh all cached data using bulk reads
         */
        private void refreshCache() {
                loopTimer.startSection("BulkRead");

                try {
                        // Perform bulk reads from each hub
                        for (LynxModule module : allHubs) {
                                LynxModuleData moduleData = moduleDataCache.get(module);
                                if (moduleData == null)
                                        continue;

                                // Read voltage
                                moduleData.voltage = module.getInputVoltage();

                                // Get bulk data in one transaction
                                LynxModule.BulkData bulkData = module.getBulkData();

                                // Cache encoder positions and velocities
                                for (int port = 0; port < 4; port++) {
                                        moduleData.encoderPositions.put(port, bulkData.getMotorCurrentPosition(port));
                                        moduleData.motorVelocities.put(port, bulkData.getMotorVelocity(port));
                                        moduleData.motorCurrents.put(port, bulkData.getMotorCurrent(port));
                                }

                                moduleData.timestamp = System.nanoTime();
                        }

                        // Update motor cache from bulk data
                        for (Map.Entry<String, MotorData> entry : motorDataCache.entrySet()) {
                                DcMotorEx motor = hardwareMap.get(DcMotorEx.class, entry.getKey());
                                LynxModule module = (LynxModule) motor.getController();
                                LynxModuleData moduleData = moduleDataCache.get(module);

                                if (moduleData != null) {
                                        int port = motor.getPortNumber();
                                        MotorData data = entry.getValue();
                                        data.position = moduleData.encoderPositions.getOrDefault(port, 0);
                                        data.velocity = moduleData.motorVelocities.getOrDefault(port, 0.0);
                                        data.current = moduleData.motorCurrents.getOrDefault(port, 0.0);
                                        data.timestamp = moduleData.timestamp;
                                }
                        }

                        // Update IMU data
                        for (Map.Entry<String, IMUData> entry : imuDataCache.entrySet()) {
                                IMU imu = hardwareMap.get(IMU.class, entry.getKey());
                                IMUData data = entry.getValue();
                                data.heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                                data.roll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
                                data.pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
                                data.timestamp = System.nanoTime();
                        }

                        cacheValid = true;
                        lastCacheTime = System.nanoTime();

                } catch (LynxNackException e) {
                        // Handle communication errors
                        cacheValid = false;
                }

                loopTimer.endSection("BulkRead");
        }

        /**
         * Get the singleton instance of BulkReadManager
         */
        public static synchronized BulkReadManager getInstance(HardwareMap hardwareMap) {
                if (instance == null) {
                        instance = new BulkReadManager(hardwareMap);
                }
                return instance;
        }

        /**
         * Clear all hub caches and invalidate cached data
         */
        public void clearCache() {
                loopTimer.startSection("ClearCache");
                for (LynxModule module : allHubs) {
                        module.clearBulkCache();
                }
                cacheValid = false;
                lastCacheTime = 0;
                loopTimer.endSection("ClearCache");
        }

        /**
         * Reset manager state
         */
        public static void reset() {
                if (instance != null) {
                        instance.motorDataCache.clear();
                        instance.imuDataCache.clear();
                        instance.moduleDataCache.clear();
                        instance = null;
                }
        }

        /**
         * Get loop timer for performance monitoring
         */
        public LoopTimer getLoopTimer() {
                return loopTimer;
        }

        /**
         * Add a motor to be monitored
         */
        public void addMotor(DcMotorEx motor) {
                motorDataCache.putIfAbsent(motor.getDeviceName(), new MotorData());
        }

        /**
         * Add an IMU to be monitored
         */
        public void addIMU(IMU imu) {
                imuDataCache.putIfAbsent(imu.getDeviceName(), new IMUData());
        }
}
