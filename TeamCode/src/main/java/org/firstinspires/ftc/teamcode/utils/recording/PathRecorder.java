package org.firstinspires.ftc.teamcode.utils.recording;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.timing.LoopTimer;
import com.example.meepmeeptesting.shared.RobotState;

import java.io.*;
import java.util.ArrayList;
import java.util.List;
import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Records and replays robot paths for autonomous development and debugging.
 * Features:
 * - Path recording with timestamps
 * - State transitions recording
 * - JSON file storage and loading
 * - Path comparison and validation
 */
public class PathRecorder {
        private final LoopTimer loopTimer;
        private final List<PathPoint> recordedPoints;
        private final String recordingsDir;
        private long startTime;
        private boolean isRecording;

        public PathRecorder(LoopTimer loopTimer) {
                this.loopTimer = loopTimer;
                this.recordedPoints = new ArrayList<>();
                this.recordingsDir = AppUtil.ROOT_FOLDER + "/PathRecordings/";
                createRecordingsDirectory();
        }

        /**
         * Start recording a new path
         */
        public void startRecording() {
                recordedPoints.clear();
                startTime = System.currentTimeMillis();
                isRecording = true;
        }

        /**
         * Record current robot position and state
         */
        public void recordPoint(Pose2d pose, RobotState state) {
                if (!isRecording)
                        return;

                long timestamp = System.currentTimeMillis() - startTime;
                recordedPoints.add(new PathPoint(pose, state, timestamp));
        }

        /**
         * Stop recording and save path to file
         */
        public void stopRecording() {
                if (!isRecording)
                        return;
                isRecording = false;

                if (!recordedPoints.isEmpty()) {
                        saveRecording(generateFilename());
                }
        }

        /**
         * Save recorded path to JSON file
         */
        private void saveRecording(String filename) {
                try {
                        JSONObject recording = new JSONObject();
                        JSONArray points = new JSONArray();

                        for (PathPoint point : recordedPoints) {
                                JSONObject pointJson = new JSONObject();
                                pointJson.put("timestamp", point.timestamp);
                                pointJson.put("x", point.pose.position.x);
                                pointJson.put("y", point.pose.position.y);
                                pointJson.put("heading", point.pose.heading);
                                pointJson.put("state", point.state.name());
                                points.put(pointJson);
                        }

                        recording.put("points", points);
                        recording.put("totalTime", recordedPoints.get(recordedPoints.size() - 1).timestamp);

                        FileWriter writer = new FileWriter(recordingsDir + filename);
                        writer.write(recording.toString(2));
                        writer.close();

                } catch (Exception e) {
                        FtcDashboard.getInstance().addLine("Failed to save recording: " + e.getMessage());
                }
        }

        /**
         * Load recorded path from file
         */
        public List<PathPoint> loadRecording(String filename) {
                List<PathPoint> points = new ArrayList<>();

                try {
                        BufferedReader reader = new BufferedReader(
                                        new FileReader(recordingsDir + filename));
                        StringBuilder content = new StringBuilder();
                        String line;
                        while ((line = reader.readLine()) != null) {
                                content.append(line);
                        }
                        reader.close();

                        JSONObject recording = new JSONObject(content.toString());
                        JSONArray pointsJson = recording.getJSONArray("points");

                        for (int i = 0; i < pointsJson.length(); i++) {
                                JSONObject pointJson = pointsJson.getJSONObject(i);
                                points.add(new PathPoint(
                                                new Pose2d(
                                                                pointJson.getDouble("x"),
                                                                pointJson.getDouble("y"),
                                                                pointJson.getDouble("heading")),
                                                RobotState.valueOf(pointJson.getString("state")),
                                                pointJson.getLong("timestamp")));
                        }

                } catch (Exception e) {
                        FtcDashboard.getInstance().addLine("Failed to load recording: " + e.getMessage());
                }

                return points;
        }

        /**
         * Compare recorded path to planned path
         */
        public PathComparison comparePaths(List<PathPoint> recorded, List<PathPoint> planned) {
                double maxPositionError = 0.0;
                double maxHeadingError = 0.0;
                double avgPositionError = 0.0;
                double avgHeadingError = 0.0;

                // Interpolate paths to match timestamps
                List<PathPoint> interpolatedPlanned = interpolatePath(planned, recorded);

                for (int i = 0; i < recorded.size(); i++) {
                        PathPoint actual = recorded.get(i);
                        PathPoint expected = interpolatedPlanned.get(i);

                        double posError = actual.pose.position.distTo(expected.pose.position);
                        double headError = Math.abs(actual.pose.heading - expected.pose.heading);

                        maxPositionError = Math.max(maxPositionError, posError);
                        maxHeadingError = Math.max(maxHeadingError, headError);
                        avgPositionError += posError;
                        avgHeadingError += headError;
                }

                avgPositionError /= recorded.size();
                avgHeadingError /= recorded.size();

                return new PathComparison(
                                maxPositionError, maxHeadingError,
                                avgPositionError, avgHeadingError);
        }

        private List<PathPoint> interpolatePath(List<PathPoint> path, List<PathPoint> timestamps) {
                List<PathPoint> interpolated = new ArrayList<>();

                for (PathPoint timePoint : timestamps) {
                        interpolated.add(interpolatePoint(path, timePoint.timestamp));
                }

                return interpolated;
        }

        private PathPoint interpolatePoint(List<PathPoint> path, long timestamp) {
                if (timestamp <= path.get(0).timestamp)
                        return path.get(0);
                if (timestamp >= path.get(path.size() - 1).timestamp)
                        return path.get(path.size() - 1);

                // Find surrounding points
                int i = 0;
                while (i < path.size() - 1 && path.get(i + 1).timestamp < timestamp)
                        i++;

                PathPoint start = path.get(i);
                PathPoint end = path.get(i + 1);

                // Linear interpolation
                double t = (timestamp - start.timestamp) /
                                (double) (end.timestamp - start.timestamp);

                return new PathPoint(
                                new Pose2d(
                                                start.pose.position.x
                                                                + t * (end.pose.position.x - start.pose.position.x),
                                                start.pose.position.y
                                                                + t * (end.pose.position.y - start.pose.position.y),
                                                start.pose.heading + t * (end.pose.heading - start.pose.heading)),
                                start.state,
                                timestamp);
        }

        private String generateFilename() {
                return String.format("path_recording_%d.json", System.currentTimeMillis());
        }

        private void createRecordingsDirectory() {
                File directory = new File(recordingsDir);
                if (!directory.exists()) {
                        directory.mkdirs();
                }
        }

        public static class PathPoint {
                public final Pose2d pose;
                public final RobotState state;
                public final long timestamp;

                public PathPoint(Pose2d pose, RobotState state, long timestamp) {
                        this.pose = pose;
                        this.state = state;
                        this.timestamp = timestamp;
                }
        }

        public static class PathComparison {
                public final double maxPositionError;
                public final double maxHeadingError;
                public final double avgPositionError;
                public final double avgHeadingError;

                public PathComparison(
                                double maxPositionError,
                                double maxHeadingError,
                                double avgPositionError,
                                double avgHeadingError) {
                        this.maxPositionError = maxPositionError;
                        this.maxHeadingError = maxHeadingError;
                        this.avgPositionError = avgPositionError;
                        this.avgHeadingError = avgHeadingError;
                }
        }
}
