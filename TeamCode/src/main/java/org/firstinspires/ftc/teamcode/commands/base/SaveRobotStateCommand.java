package org.firstinspires.ftc.teamcode.commands.base;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Supplier;
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
    private final Map<String, Object> state;
    private final String filename;
    private Telemetry telemetry;
    private LowerSlide lowerslide;
    private UpperSlide upperslide;

    public SaveRobotStateCommand(MecanumDrive drive, LowerSlide lowerslide, UpperSlide upperslide) {
        this(drive, lowerslide, upperslide, "robot_state.txt");
    }

    public SaveRobotStateCommand(MecanumDrive drive,  LowerSlide lowerslide, UpperSlide upperslide, String filename) {
        this.drive = drive;
        this.lowerslide = lowerslide;
        this.upperslide = upperslide;
        this.filename = filename;
        this.state = new HashMap<>();
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        addRequirement(lowerslide);
        addRequirement(upperslide);
    }


    @Override
    public void initialize() {
        // Capture current robot state
        Pose2d currentPose = drive.localizer.getPose();
        state.put("drive/pose/x", currentPose.position.x);
        state.put("drive/pose/y", currentPose.position.y);
        state.put("drive/pose/heading", currentPose.heading.toDouble());
        state.put("lowerslide/position", lowerslide.getCurrentPosition());
        state.put("upperslide/position", upperslide.getCurrentPosition());
        state.put("timestamp", System.currentTimeMillis());
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (telemetry != null) {
            telemetry.addData("SaveState", "Capturing robot state...");
            telemetry.addLine("--- Written State ---");
            for (Map.Entry<String, Object> entry : state.entrySet()) {
                telemetry.addData("State/"+entry.getKey(), entry.getValue());
            }
        }
        try {
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
        } catch (IOException e) {
            telemetry.addData("SaveState", "Error: " + e.getMessage());
            telemetry.update();
        }
        telemetry.addData("SaveState", "Complete");
        telemetry.update();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}