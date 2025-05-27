package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;

/**
 * Base OpMode class with performance monitoring and bulk reads.
 * All OpModes should extend this class to benefit from optimized hardware reads
 * and performance tracking.
 */
public abstract class BaseOpMode extends OpMode {
        protected final LoopTimer loopTimer = new LoopTimer();
        protected BulkReadManager bulkReader;

        @Override
        public void init() {
                bulkReader = BulkReadManager.getInstance(hardwareMap);
                loopTimer.reset();

                // Call child class initialization
                initialize();
        }

        /**
         * Child classes should override this method for initialization
         * instead of init()
         */
        protected abstract void initialize();

        @Override
        public void loop() {
                loopTimer.startLoop();
                bulkReader.clearCache();

                // Time the main update cycle
                loopTimer.startSection("update");
                update();
                loopTimer.endSection("update");

                // Report performance metrics
                loopTimer.startSection("telemetry");
                updateTelemetry();
                loopTimer.endSection("telemetry");

                loopTimer.endLoop();
        }

        /**
         * Child classes should override this method for their main loop
         * instead of loop()
         */
        protected abstract void update();

        /**
         * Update telemetry with performance metrics and custom data
         */
        protected void updateTelemetry() {
                // Performance metrics
                loopTimer.reportTelemetry(telemetry);

                // Custom telemetry from child class
                addCustomTelemetry();

                telemetry.update();
        }

        /**
         * Child classes can override to add custom telemetry data
         */
        protected void addCustomTelemetry() {
                // Default implementation does nothing
        }

        @Override
        public void stop() {
                BulkReadManager.reset();
                super.stop();
        }
}
