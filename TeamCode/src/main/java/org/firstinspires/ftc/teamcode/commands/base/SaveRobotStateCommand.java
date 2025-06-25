package org.firstinspires.ftc.teamcode.commands.base;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;
import org.firstinspires.ftc.teamcode.utils.ClawController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collection;
import java.util.Collections;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class SaveRobotStateCommand extends CommandBase {
    private final MecanumDrive drive;
    private final LowerSlide lowerslide;
    private final UpperSlide upperslide;
    private final Map<String, Object> state;
    private final String filename;
    private boolean saveComplete = false;
    private Telemetry telemetry;

    public SaveRobotStateCommand(MecanumDrive drive, LowerSlide lowerslide, UpperSlide upperslide) {
        this(drive, lowerslide, upperslide, "robot_state.txt");
    }

    public SaveRobotStateCommand(MecanumDrive drive, LowerSlide lowerslide, UpperSlide upperslide, String filename) {
        this.drive = drive;
        this.lowerslide = lowerslide;
        this.upperslide = upperslide;
        this.filename = filename;
        this.state = new HashMap<>();
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
    }


    @Override
    public void initialize() {
        state.clear();
        saveComplete = false;

        // Capture current robot state
        Pose2d currentPose = drive.localizer.getPose();
        state.put("drive/pose/x", currentPose.position.x);
        state.put("drive/pose/y", currentPose.position.y);
        state.put("drive/pose/heading", currentPose.heading.toDouble());
        state.put("lowerslide/position", lowerslide.getCurrentPosition());
        state.put("upperslide/position", upperslide.getCurrentPosition());
        state.put("timestamp", System.currentTimeMillis());

        if (telemetry != null) {
            telemetry.addData("SaveState", "Capturing robot state...");
            telemetry.update();
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (!saveComplete) {
            try {
                saveStateToFile();
                saveComplete = true;

                if (telemetry != null) {
                    telemetry.addData("SaveState", "State saved successfully to " + filename);
                    telemetry.update();
                }

                if (packet != null) {
                    packet.put("SaveState", "Success");
                }
            } catch (IOException e) {
                saveComplete = true; // End command even on error

                if (telemetry != null) {
                    telemetry.addData("SaveState", "Error: " + e.getMessage());
                    telemetry.update();
                }

                if (packet != null) {
                    packet.put("SaveState", "Error: " + e.getMessage());
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return saveComplete;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && telemetry != null) {
            telemetry.addData("SaveState", "Command interrupted");
            telemetry.update();
        }
    }

    private void saveStateToFile() throws IOException {
        File file = new File("/sdcard/FIRST/robot_states/" + filename);

        // Create directory if it doesn't exist
        file.getParentFile().mkdirs();

        try (FileWriter writer = new FileWriter(file)) {
            writer.write("# Robot State Save File\n");
            writer.write("# Timestamp: " + System.currentTimeMillis() + "\n");
            writer.write("# Date: " + new java.util.Date().toString() + "\n\n");

            for (Map.Entry<String, Object> entry : state.entrySet()) {
                writer.write(entry.getKey() + "=" + entry.getValue().toString() + "\n");
            }
        }
    }

    public Map<String, Object> getState() {
        return new HashMap<>(state);
    }
}