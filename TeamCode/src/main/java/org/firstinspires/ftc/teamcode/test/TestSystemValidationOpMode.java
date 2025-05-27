package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;
import org.firstinspires.ftc.teamcode.utils.validation.SystemValidator;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Test OpMode for validating robot systems before matches.
 * Performs comprehensive system checks and reports status.
 */
@TeleOp(name = "Test: System Validation", group = "Test")
public class TestSystemValidationOpMode extends BaseOpMode {
        private SystemValidator validator;
        private FtcDashboard dashboard;
        private boolean lastAState = false;
        private boolean lastBState = false;

        @Override
        protected void initialize() {
                validator = new SystemValidator(hardwareMap, telemetry, loopTimer);
                dashboard = FtcDashboard.getInstance();
                displayControls();
        }

        private void displayControls() {
                telemetry.addLine("=== System Validation Controls ===");
                telemetry.addData("A Button", "Start Validation");
                telemetry.addData("B Button", "Reset/Restart");
                telemetry.update();
        }

        @Override
        protected void update() {
                // Start validation with A
                boolean currentAState = gamepad1.a;
                if (currentAState && !lastAState) {
                        if (!validator.isValidating()) {
                                validator.startValidation();
                        }
                }
                lastAState = currentAState;

                // Reset with B
                boolean currentBState = gamepad1.b;
                if (currentBState && !lastBState) {
                        initialize();
                }
                lastBState = currentBState;

                // Update validation system
                validator.update();

                // Update dashboard
                updateDashboard();
        }

        private void updateDashboard() {
                TelemetryPacket packet = new TelemetryPacket();

                // Add validation status
                packet.put("validation_running", validator.isValidating());
                packet.put("validation_complete", validator.allChecksPassed());

                // Add loop timing
                packet.put("loop_time_ms", loopTimer.getLoopTime());
                packet.put("avg_loop_time_ms", loopTimer.getAverageLoopTime());

                dashboard.sendTelemetryPacket(packet);
        }

        @Override
        protected void addCustomTelemetry() {
                if (!validator.isValidating() && validator.allChecksPassed()) {
                        telemetry.addLine("✓ All Systems Go!");
                        telemetry.addLine("Press B to restart validation");
                } else if (!validator.isValidating()) {
                        telemetry.addLine("Press A to start system validation");
                }
        }
}
