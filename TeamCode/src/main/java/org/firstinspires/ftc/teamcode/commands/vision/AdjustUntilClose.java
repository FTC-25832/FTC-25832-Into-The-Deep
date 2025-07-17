package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.function.Supplier;

public class AdjustUntilClose extends CommandBase {
    private final MecanumDrive drive;

    private final LowerSlide lowSlide;
    private final Runnable disableDriveControl;
    private final Runnable enableDriveControl;
    private final Runnable updateDetectorResult;
    private final Supplier<Double> txSupplier;
    private final Supplier<Double> tySupplier;
    private final Supplier<Double> pxSupplier;
    private final Supplier<Double> pySupplier;

    private DistanceAdjustLUTX xAdjustCommand;
    private DistanceAdjustLUTY yAdjustCommand;

    private boolean isXAdjusting = false;
    private boolean isYAdjusting = false;
    private boolean isFinished = false;
    private boolean firstTime = true;
    private static final double TOLERANCE = ConfigVariables.Camera.TOLERANCE; // ±0.25 degrees tolerance
    private static final int MAX_ATTEMPTS = 4; // Maximum adjustment iterations
    private static final double ADJUSTMENT_DELAY = ConfigVariables.Camera.ADJUSTMENT_DELAY; // 500ms between adjustments
    private static final double SUBCOMMAND_TIMEOUT_SECONDS = 3.0; // Timeout for each adjustment phase
    private ElapsedTime subCommandTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private String lastSubCommandStatus = "";

    private int attemptCount = 0;
    private ElapsedTime timeBetweenAdjustments;

    // Local flags to track sub-command error/status for this execute cycle
    private boolean xHadVisionError = false;
    private boolean xHadActionStatus = false;
    private boolean yHadVisionError = false;

    public AdjustUntilClose(MecanumDrive drive, LowerSlide lowSlide,
            Supplier<Double> txSupplier, Supplier<Double> tySupplier, Supplier<Double> pxSupplier,
            Supplier<Double> pySupplier,
            Runnable disableDriveControl, Runnable enableDriveControl, Runnable UpdateDetectorResult) {
        // Defensive null checks
        if (drive == null || lowSlide == null || txSupplier == null || tySupplier == null || pxSupplier == null
                || pySupplier == null ||
                disableDriveControl == null || enableDriveControl == null || UpdateDetectorResult == null) {
            throw new IllegalArgumentException("No argument to AdjustUntilClose may be null");
        }
        this.drive = drive;
        this.lowSlide = lowSlide;
        this.txSupplier = txSupplier;
        this.tySupplier = tySupplier;
        this.pxSupplier = pxSupplier;
        this.pySupplier = pySupplier;
        this.disableDriveControl = disableDriveControl;
        this.enableDriveControl = enableDriveControl;
        this.updateDetectorResult = UpdateDetectorResult;

        // Create the adjustment commands
        this.xAdjustCommand = new DistanceAdjustLUTX(drive, txSupplier, tySupplier, pxSupplier, pySupplier,
                disableDriveControl, enableDriveControl);
        this.yAdjustCommand = new DistanceAdjustLUTY(lowSlide, tySupplier);

        this.timeBetweenAdjustments = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.subCommandTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        // Add requirements
        addRequirement(lowSlide);
    }

    @Override
    public void initialize() {
        isXAdjusting = false;
        isYAdjusting = false;
        isFinished = false;
        attemptCount = 0;
        timeBetweenAdjustments.reset();
        subCommandTimer.reset();
        lastSubCommandStatus = "";
        // Ensure sub-commands are reset if reused
        xAdjustCommand.end(true);
        yAdjustCommand.end(true);
    }

    @Override
    public void execute(TelemetryPacket packet) {
        updateDetectorResult.run();
        double tx = safeGet(txSupplier);
        double ty = safeGet(tySupplier);

        packet.put("AdjustUntilClose/tx", tx);
        packet.put("AdjustUntilClose/ty", ty);
        packet.put("AdjustUntilClose/attempts", attemptCount);
        packet.put("AdjustUntilClose/lastSubCommandStatus", lastSubCommandStatus);

        // Check if we're already aligned within tolerance
        boolean txWithinTolerance = Math.abs(tx) <= TOLERANCE;
        boolean tyWithinTolerance = Math.abs(ty) <= TOLERANCE;

        packet.put("AdjustUntilClose/txWithinTolerance", txWithinTolerance);
        packet.put("AdjustUntilClose/tyWithinTolerance", tyWithinTolerance);

        if (txWithinTolerance && tyWithinTolerance) {
            isFinished = true;
            packet.put("AdjustUntilClose/status", "ALIGNED");
            return;
        }

        // Check if we've reached maximum adjustment attempts
        if (attemptCount >= MAX_ATTEMPTS) {
            isFinished = true;
            packet.put("AdjustUntilClose/status", "MAX_ATTEMPTS_REACHED");
            return;
        }

        // State machine for adjustments
        if (!isXAdjusting && !isYAdjusting) {
            // If we're not currently adjusting, start a new adjustment cycle
            if (timeBetweenAdjustments.seconds() >= ADJUSTMENT_DELAY || firstTime) {
                firstTime = false;
                if (!txWithinTolerance) {
                    // Start X adjustment first
                    xAdjustCommand.initialize();
                    isXAdjusting = true;
                    subCommandTimer.reset();
                    lastSubCommandStatus = "";
                    packet.put("AdjustUntilClose/status", "ADJUSTING_X");
                } else if (!tyWithinTolerance) {
                    // Start Y adjustment
                    yAdjustCommand.initialize();
                    isYAdjusting = true;
                    subCommandTimer.reset();
                    lastSubCommandStatus = "";
                    packet.put("AdjustUntilClose/status", "ADJUSTING_Y");
                }
            } else {
                packet.put("AdjustUntilClose/status", "WAITING_BETWEEN_ADJUSTMENTS");
                packet.put("AdjustUntilClose/timeRemaining", ADJUSTMENT_DELAY - timeBetweenAdjustments.seconds());
            }
        }

        // Run X adjustment if active
        if (isXAdjusting) {
            xAdjustCommand.execute(packet);
            // Timeout logic for X adjustment
            if (subCommandTimer.seconds() > SUBCOMMAND_TIMEOUT_SECONDS) {
                isXAdjusting = false;
                lastSubCommandStatus = "X_TIMEOUT";
                packet.put("AdjustUntilClose/status", "X_TIMEOUT");
                timeBetweenAdjustments.reset();
                attemptCount++;
            } else if (xAdjustCommand.isFinished()) {
                isXAdjusting = false;
                // Check for sub-command error/timeout via local flags
                // (You must set xHadVisionError/xHadActionStatus in DistanceAdjustLUTX when you
                // put those keys)
                if (xHadVisionError) {
                    lastSubCommandStatus = "X_ERROR: vision/error";
                } else if (xHadActionStatus) {
                    lastSubCommandStatus = "X_STATUS: DistanceAdjustLUTX/actionStatus";
                } else {
                    lastSubCommandStatus = "X_DONE";
                }
                // If X is finished and Y needs adjustment, start Y
                if (!tyWithinTolerance) {
                    yAdjustCommand.initialize();
                    isYAdjusting = true;
                    subCommandTimer.reset();
                    packet.put("AdjustUntilClose/status", "X_DONE_STARTING_Y");
                } else {
                    // X is done and Y is within tolerance
                    timeBetweenAdjustments.reset();
                    attemptCount++;
                    packet.put("AdjustUntilClose/status", "X_DONE_Y_OK");
                }
            }
        }

        // Run Y adjustment if active
        if (isYAdjusting) {
            yAdjustCommand.execute(packet);
            // Timeout logic for Y adjustment
            if (subCommandTimer.seconds() > SUBCOMMAND_TIMEOUT_SECONDS) {
                isYAdjusting = false;
                lastSubCommandStatus = "Y_TIMEOUT";
                packet.put("AdjustUntilClose/status", "Y_TIMEOUT");
                timeBetweenAdjustments.reset();
                attemptCount++;
            } else if (yAdjustCommand.isFinished()) {
                isYAdjusting = false;
                if (yHadVisionError) {
                    lastSubCommandStatus = "Y_ERROR: vision/error";
                } else {
                    lastSubCommandStatus = "Y_DONE";
                }
                timeBetweenAdjustments.reset();
                attemptCount++;
                packet.put("AdjustUntilClose/status", "Y_DONE");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if (isXAdjusting) {
            xAdjustCommand.end(interrupted);
        }
        if (isYAdjusting) {
            yAdjustCommand.end(interrupted);
        }

        // Ensure drive control is enabled when we're done
        enableDriveControl.run();
    }

    // Helper to safely get supplier values (null-safe, returns 0 if null)
    private double safeGet(Supplier<Double> supplier) {
        try {
            Double val = supplier.get();
            return val != null ? val : 0.0;
        } catch (Exception e) {
            return 0.0;
        }
    }
}