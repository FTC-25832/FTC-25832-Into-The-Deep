package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.subsystems.slides.UpperSlide;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class ReadRobotStateCommand extends CommandBase {
    private final MecanumDrive drive;
    private final LowerSlide lowerslide;
    private final UpperSlide upperslide;
    private final String filename;
    private final boolean restoreState;
    private Map<String, String> loadedState;
    private boolean readComplete = false;
    private Telemetry telemetry;

    public ReadRobotStateCommand(MecanumDrive drive, LowerSlide lowerslide, UpperSlide upperslide) {
        this(drive, lowerslide, upperslide, "robot_state.txt", true);
    }

    public ReadRobotStateCommand(MecanumDrive drive, LowerSlide lowerslide, UpperSlide upperslide, String filename) {
        this(drive, lowerslide, upperslide, filename, true);
    }

    public ReadRobotStateCommand(MecanumDrive drive, LowerSlide lowerslide, UpperSlide upperslide, String filename, boolean restoreState) {
        this.drive = drive;
        this.lowerslide = lowerslide;
        this.upperslide = upperslide;
        this.filename = filename;
        this.restoreState = restoreState;
        this.loadedState = new HashMap<>();
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
    }


    @Override
    public void initialize() {
        loadedState.clear();
        readComplete = false;

        if (telemetry != null) {
            telemetry.addData("ReadState", "Loading robot state from " + filename);
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (!readComplete) {
            try {
                loadStateFromFile();

                if (restoreState) {
                    restoreRobotState();
                }

                readComplete = true;

                if (telemetry != null) {
                    telemetry.addData("ReadState", "State loaded successfully");
                    displayLoadedState();
                }

                if (packet != null) {
                    packet.put("ReadState", "Success");
                    for (Map.Entry<String, String> entry : loadedState.entrySet()) {
                        packet.put("State/" + entry.getKey(), entry.getValue());
                    }
                }

            } catch (IOException e) {
                readComplete = true; // End command even on error

                if (telemetry != null) {
                    telemetry.addData("ReadState", "Error: " + e.getMessage());

                }

                if (packet != null) {
                    packet.put("ReadState", "Error: " + e.getMessage());
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return readComplete;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && telemetry != null) {
            telemetry.addData("ReadState", "Command interrupted");
        }
        telemetry.update();
    }

    private void loadStateFromFile() throws IOException {
        File file = new File("/sdcard/FIRST/robot_states/" + filename);

        if (!file.exists()) {
            throw new IOException("State file not found: " + filename);
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.trim();

                // Skip comments and empty lines
                if (line.startsWith("#") || line.isEmpty()) {
                    continue;
                }

                // Parse key=value pairs
                int equalIndex = line.indexOf('=');
                if (equalIndex > 0) {
                    String key = line.substring(0, equalIndex).trim();
                    String value = line.substring(equalIndex + 1).trim();
                    loadedState.put(key, value);
                }
            }
        }
    }

    private void restoreRobotState() {
        try {
            // Restore drive pose
            if (loadedState.containsKey("drive/pose/x") && loadedState.containsKey("drive/pose/y") && loadedState.containsKey("drive/pose/heading")) {

                double x = Double.parseDouble(loadedState.get("drive/pose/x"));
                double y = Double.parseDouble(loadedState.get("drive/pose/y"));
                double heading = Double.parseDouble(loadedState.get("drive/pose/heading"));

                Pose2d restoredPose = new Pose2d(new Vector2d(x, y), Rotation2d.fromDouble(heading));
                drive.localizer.setPose(restoredPose);

                if (telemetry != null) {
                    telemetry.addData("Restored Pose", "X: %.2f, Y: %.2f, H: %.2f°", x, y, Math.toDegrees(heading));
                }
            }

            // Restore slide positions
            if (loadedState.containsKey("lowerslide/position")) {
                double lowerPosition = Double.parseDouble(loadedState.get("lowerslide/position"));
                lowerslide.setTickOffset((int) lowerPosition);

                if (telemetry != null) {
                    telemetry.addData("Restored Lower Slide", lowerPosition);
                }
            }

            if (loadedState.containsKey("upperslide/position")) {
                double upperPosition = Double.parseDouble(loadedState.get("upperslide/position"));
                upperslide.setTickOffset((int) upperPosition);

                if (telemetry != null) {
                    telemetry.addData("Restored Upper Slide", upperPosition);
                }
            }

        } catch (NumberFormatException e) {
            if (telemetry != null) {
                telemetry.addData("Restore Error", "Invalid number format in state file");
            }
        }
    }

    private void displayLoadedState() {
        if (telemetry == null) return;

        telemetry.addLine("--- Loaded State ---");
        for (Map.Entry<String, String> entry : loadedState.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }

    public Map<String, String> getLoadedState() {
        return new HashMap<>(loadedState);
    }

    public String getStateValue(String key) {
        return loadedState.get(key);
    }

    public boolean hasStateValue(String key) {
        return loadedState.containsKey(key);
    }
}