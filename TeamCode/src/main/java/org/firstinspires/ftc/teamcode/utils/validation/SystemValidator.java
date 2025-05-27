package org.firstinspires.ftc.teamcode.utils.validation;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.utils.hardware.BulkReadManager;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;
import org.firstinspires.ftc.teamcode.vision.VisionStateManager;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.constants.TimingConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Pre-match system validation utility.
 * Performs comprehensive checks of robot systems before matches.
 */
public class SystemValidator {
        private final HardwareMap hardwareMap;
        private final Telemetry telemetry;
        private final List<ValidationCheck> checks;
        private final LoopTimer loopTimer;

        private boolean isValidating = false;
        private int currentCheckIndex = 0;
        private boolean lastCheckPassed = true;
        private String lastError = "";

        public SystemValidator(HardwareMap hardwareMap, Telemetry telemetry, LoopTimer loopTimer) {
                this.hardwareMap = hardwareMap;
                this.telemetry = telemetry;
                this.loopTimer = loopTimer;
                this.checks = new ArrayList<>();
                setupChecks();
        }

        private void setupChecks() {
                // Hardware initialization checks
                checks.add(new ValidationCheck(
                                "IMU Initialization",
                                () -> {
                                        IMU imu = hardwareMap.get(IMU.class, "imu");
                                        return imu != null && imu.isGyroCalibrated();
                                },
                                TimingConstants.IMU_INIT_TIMEOUT));

                // Motor checks
                checks.add(new ValidationCheck(
                                "Drive Motors",
                                () -> {
                                        try {
                                                DcMotor fl = hardwareMap.get(DcMotor.class, "frontLeft");
                                                DcMotor fr = hardwareMap.get(DcMotor.class, "frontRight");
                                                DcMotor bl = hardwareMap.get(DcMotor.class, "backLeft");
                                                DcMotor br = hardwareMap.get(DcMotor.class, "backRight");
                                                return fl.getMotorType() != null &&
                                                                fr.getMotorType() != null &&
                                                                bl.getMotorType() != null &&
                                                                br.getMotorType() != null;
                                        } catch (Exception e) {
                                                return false;
                                        }
                                },
                                TimingConstants.SYSTEM_CHECK_TIMEOUT));

                // Bulk read check
                checks.add(new ValidationCheck(
                                "Bulk Read System",
                                () -> {
                                        try {
                                                BulkReadManager manager = BulkReadManager.getInstance(hardwareMap);
                                                manager.clearCache();
                                                return true;
                                        } catch (Exception e) {
                                                return false;
                                        }
                                },
                                TimingConstants.SYSTEM_CHECK_TIMEOUT));

                // Vision system check
                checks.add(new ValidationCheck(
                                "Vision System",
                                () -> {
                                        try {
                                                VisionStateManager vision = new VisionStateManager(loopTimer);
                                                return !vision.getCurrentState().isErrorState();
                                        } catch (Exception e) {
                                                return false;
                                        }
                                },
                                TimingConstants.VISION_DETECTION_TIMEOUT));

                // Localization check
                checks.add(new ValidationCheck(
                                "Localization",
                                () -> {
                                        try {
                                                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d());
                                                drive.updatePoseEstimate();
                                                return true;
                                        } catch (Exception e) {
                                                return false;
                                        }
                                },
                                TimingConstants.SYSTEM_CHECK_TIMEOUT));

                // Loop timing check
                checks.add(new ValidationCheck(
                                "Loop Performance",
                                () -> {
                                        loopTimer.update();
                                        double loopTime = loopTimer.getLoopTime();
                                        return loopTime < TimingConstants.LOOP_TIME_WARNING_THRESHOLD;
                                },
                                TimingConstants.SYSTEM_CHECK_TIMEOUT));
        }

        public void startValidation() {
                isValidating = true;
                currentCheckIndex = 0;
                lastCheckPassed = true;
                lastError = "";
                updateTelemetry();
        }

        public void update() {
                if (!isValidating)
                        return;

                if (currentCheckIndex < checks.size()) {
                        ValidationCheck currentCheck = checks.get(currentCheckIndex);

                        // Run check with timeout
                        long startTime = System.currentTimeMillis();
                        try {
                                lastCheckPassed = currentCheck.check.get();
                                lastError = "";
                        } catch (Exception e) {
                                lastCheckPassed = false;
                                lastError = e.getMessage();
                        }

                        // Check timeout
                        if (System.currentTimeMillis() - startTime > currentCheck.timeoutMs) {
                                lastCheckPassed = false;
                                lastError = "Check timed out";
                        }

                        if (!lastCheckPassed) {
                                isValidating = false;
                        } else {
                                currentCheckIndex++;
                        }

                        updateTelemetry();
                }
        }

        public boolean isValidating() {
                return isValidating;
        }

        public boolean allChecksPassed() {
                return !isValidating && currentCheckIndex == checks.size() && lastCheckPassed;
        }

        private void updateTelemetry() {
                telemetry.addLine("=== System Validation ===");

                // Show progress
                telemetry.addData("Status", isValidating ? "Running" : (lastCheckPassed ? "Complete" : "Failed"));
                telemetry.addData("Progress", String.format("%d/%d", currentCheckIndex, checks.size()));

                // Show current/last check
                if (currentCheckIndex < checks.size()) {
                        ValidationCheck current = checks.get(currentCheckIndex);
                        telemetry.addData("Current Check", current.name);
                }

                // Show error if any
                if (!lastError.isEmpty()) {
                        telemetry.addData("Error", lastError);
                }

                // Show all check results
                telemetry.addLine("\n=== Check Results ===");
                for (int i = 0; i < currentCheckIndex; i++) {
                        telemetry.addData(checks.get(i).name, "✓");
                }
                if (!lastCheckPassed && currentCheckIndex < checks.size()) {
                        telemetry.addData(checks.get(currentCheckIndex).name, "✗");
                }

                telemetry.update();
        }

        private static class ValidationCheck {
                public final String name;
                public final Supplier<Boolean> check;
                public final long timeoutMs;

                public ValidationCheck(String name, Supplier<Boolean> check, long timeoutMs) {
                        this.name = name;
                        this.check = check;
                        this.timeoutMs = timeoutMs;
                }
        }
}
