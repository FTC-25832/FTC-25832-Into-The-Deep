package org.firstinspires.ftc.teamcode.utils.timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

/**
 * Utility class to monitor and log loop timing statistics.
 * Integrates with both driver station telemetry and FTC Dashboard.
 */
public class LoopTimer {
        private final ElapsedTime timer = new ElapsedTime();
        private final Map<String, TimingStats> stats = new HashMap<>();
        private final FtcDashboard dashboard = FtcDashboard.getInstance();
        private long lastLoopTime = 0;
        private static final int SAMPLE_SIZE = 50; // Rolling average window size

        /**
         * Start timing a new section
         * 
         * @param sectionName Name of the section to time
         */
        public void startSection(String sectionName) {
                timer.reset();
        }

        /**
         * End timing a section and record its duration
         * 
         * @param sectionName Name of the section that was timed
         */
        public void endSection(String sectionName) {
                double duration = timer.milliseconds();
                TimingStats section = stats.computeIfAbsent(sectionName, k -> new TimingStats());
                section.addSample(duration);
        }

        /**
         * Start timing a new loop iteration
         */
        public void startLoop() {
                lastLoopTime = System.nanoTime();
        }

        /**
         * End timing the current loop iteration and record its duration
         */
        public void endLoop() {
                long currentTime = System.nanoTime();
                double loopTime = (currentTime - lastLoopTime) / 1e6; // Convert to milliseconds
                TimingStats loopStats = stats.computeIfAbsent("FullLoop", k -> new TimingStats());
                loopStats.addSample(loopTime);
        }

        /**
         * Report timing statistics to telemetry
         * 
         * @param telemetry Driver station telemetry instance
         */
        public void reportTelemetry(Telemetry telemetry) {
                telemetry.addLine("=== Loop Timing Stats ===");
                for (Map.Entry<String, TimingStats> entry : stats.entrySet()) {
                        TimingStats stat = entry.getValue();
                        telemetry.addData(entry.getKey(),
                                        "Avg: %.2fms, Min: %.2fms, Max: %.2fms",
                                        stat.getAverage(), stat.getMin(), stat.getMax());
                }

                // Also send to dashboard
                TelemetryPacket packet = new TelemetryPacket();
                for (Map.Entry<String, TimingStats> entry : stats.entrySet()) {
                        TimingStats stat = entry.getValue();
                        packet.put(entry.getKey() + "/avg", stat.getAverage());
                        packet.put(entry.getKey() + "/min", stat.getMin());
                        packet.put(entry.getKey() + "/max", stat.getMax());
                }
                dashboard.sendTelemetryPacket(packet);
        }

        /**
         * Reset all timing statistics
         */
        public void reset() {
                stats.clear();
                lastLoopTime = 0;
        }

        /**
         * Inner class to track timing statistics for a section
         */
        private static class TimingStats {
                private final double[] samples = new double[SAMPLE_SIZE];
                private int sampleIndex = 0;
                private int sampleCount = 0;
                private double min = Double.MAX_VALUE;
                private double max = Double.MIN_VALUE;
                private double sum = 0;

                public void addSample(double value) {
                        // Remove old sample from sum if we're overwriting
                        if (sampleCount == SAMPLE_SIZE) {
                                sum -= samples[sampleIndex];
                        }

                        // Add new sample
                        samples[sampleIndex] = value;
                        sum += value;

                        // Update min/max
                        min = Math.min(min, value);
                        max = Math.max(max, value);

                        // Update counters
                        sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
                        if (sampleCount < SAMPLE_SIZE)
                                sampleCount++;
                }

                public double getAverage() {
                        return sampleCount > 0 ? sum / sampleCount : 0;
                }

                public double getMin() {
                        return min == Double.MAX_VALUE ? 0 : min;
                }

                public double getMax() {
                        return max == Double.MIN_VALUE ? 0 : max;
                }
        }
}
