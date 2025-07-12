package org.firstinspires.ftc.teamcode.sensors.limelight;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

public class Limelight {
    HardwareMap hardwareMap;
    LLResult result;
    public List<LLResultTypes.DetectorResult> detectorResults;
    public LLResultTypes.DetectorResult detectorResult;
    public double[] poseResult = { 0, 0, 0, 0 }; // [x, y, z, rotationAngle]
    public List<List<Double>> outerCorners;
    public boolean available = true;
    public boolean resultAvailable = false;
    private static final Mat CAMERA_INTRINSIC_MATRIX = new Mat(3, 3, CvType.CV_64F);
    private static final MatOfDouble DISTORTION_COEFFICIENTS = new MatOfDouble(Mat.zeros(5, 1, CvType.CV_64F));
    static {
        CAMERA_INTRINSIC_MATRIX.put(0, 0,
                1221.445, 0, 637.226,
                0, 1223.398, 502.549,
                0, 0, 1);
        DISTORTION_COEFFICIENTS.put(0, 0, 0.177168, -0.457341, 0.000360, 0.002753, 0.178259);
    }
    private static final double PRISM_LENGTH = 8.5; // cm
    private static final double PRISM_WIDTH = 3.5; // cm
    private static final double CAMERA_HEIGHT = ConfigVariables.Camera.CAMERA_HEIGHT; // cm
    private static final double CAMERA_TILT_ANGLE = ConfigVariables.Camera.TILT_ANGLE; // degrees
    private static final double WIDTH_RATIO = PRISM_LENGTH;
    private static final double HEIGHT_RATIO = PRISM_WIDTH;

    public Limelight3A limelight;

    public void initialize(HardwareMap map) {
        hardwareMap = map;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight == null) {
            available = false;
            return;
        }
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
    }

    public void cameraStart() {
        limelight.start();
        limelight.reloadPipeline();
    }

    /*
     * return true if the result is valid (detected)
     */
    public boolean updateDetectorResult() {
        if(!available) return false;
        result = limelight.getLatestResult();
        if (result.isValid()) {
            detectorResults = result.getDetectorResults();
            if (detectorResults.isEmpty()) {
                resultAvailable = false;
                return false;
            }
            // get the first result whose classname is in
            // ConfigVariables.Camera.ACCEPTED_COLORS
            for (LLResultTypes.DetectorResult res : detectorResults) {
                if (Arrays.asList(ConfigVariables.Camera.ACCEPTED_COLORS).contains(res.getClassName())) {
                    detectorResult = res;
                    resultAvailable = true;
                    return true;
                }
            }
            resultAvailable = false;
            return false;
        }
        resultAvailable = false;
        return false;
    }

    public void updatePosition() {
        if (!resultAvailable || !available)
            return;

        outerCorners = detectorResult.getTargetCorners();
        if (outerCorners == null || outerCorners.size() != 4) {
            resultAvailable = false;
            return;
        }

        List<List<Double>> innerCorners = getInnerCorners(outerCorners);
        if (innerCorners.isEmpty()) {
            resultAvailable = false;
            return;
        }
        MatOfPoint2f innerCornersMat = new MatOfPoint2f(
                new Point(innerCorners.get(0).get(0), innerCorners.get(0).get(1)),
                new Point(innerCorners.get(1).get(0), innerCorners.get(1).get(1)),
                new Point(innerCorners.get(2).get(0), innerCorners.get(2).get(1)),
                new Point(innerCorners.get(3).get(0), innerCorners.get(3).get(1)));
        poseResult = estimatePrismPose(innerCornersMat);
    }

    public static List<List<Double>> getInnerCorners(List<List<Double>> outerCorners) {
        if (outerCorners == null || outerCorners.size() != 4) {
            return new ArrayList<>();
        }
        double A = outerCorners.get(1).get(0) - outerCorners.get(0).get(0);
        double B = outerCorners.get(2).get(1) - outerCorners.get(1).get(1);
        double[] solution = solveEquations(A, B);
        if (solution[1] == 0 || solution[2] == 0) {
            return new ArrayList<>();
        }
        double x = solution[0];
        double a = solution[1];
        double b = solution[2];

        double centerX = (outerCorners.get(0).get(0) + outerCorners.get(2).get(0)) / 2;
        double centerY = (outerCorners.get(0).get(1) + outerCorners.get(2).get(1)) / 2;

        List<List<Double>> innerCorners = new ArrayList<>();
        double halfWidth = a / 2;
        double halfHeight = b / 2;

        double[][] offsets = {
                { -halfWidth, -halfHeight },
                { halfWidth, -halfHeight },
                { halfWidth, halfHeight },
                { -halfWidth, halfHeight }
        };

        for (double[] offset : offsets) {
            double rotatedX = offset[0] * Math.cos(x) - offset[1] * Math.sin(x);
            double rotatedY = offset[0] * Math.sin(x) + offset[1] * Math.cos(x);

            List<Double> corner = new ArrayList<>();
            corner.add(centerX + rotatedX);
            corner.add(centerY + rotatedY);
            innerCorners.add(corner);
        }
        return innerCorners;
    }

    private static double[] solveEquations(double A, double B) {
        if (Math.abs(A) < 1e-10 && Math.abs(B) < 1e-10) {
            return new double[] { 0.0, 0.0, 0.0 };
        }
        double numerator = HEIGHT_RATIO * A - WIDTH_RATIO * B;
        double denominator = HEIGHT_RATIO * B - WIDTH_RATIO * A;
        double x;
        if (Math.abs(numerator) < 1e-10) {
            x = 0.0;
        } else if (Math.abs(denominator) < 1e-10) {
            x = Math.PI / 2;
        } else {
            x = Math.atan2(numerator, denominator);
        }
        double denominatorK1 = WIDTH_RATIO * Math.cos(x) + HEIGHT_RATIO * Math.sin(x);
        double denominatorK2 = WIDTH_RATIO * Math.sin(x) + HEIGHT_RATIO * Math.cos(x);
        double k;
        if (Math.abs(denominatorK1) > 1e-10) {
            k = A / denominatorK1;
        } else if (Math.abs(denominatorK2) > 1e-10) {
            k = B / denominatorK2;
        } else {
            return new double[] { 0.0, 0.0, 0.0 };
        }
        double a = WIDTH_RATIO * k;
        double b = HEIGHT_RATIO * k;
        return new double[] { x, a, b };
    }

    /**
     * Estimates the pose (position and rotation) of a rectangular prism from
     * observed image points.
     *
     * @param imagePoints 2D coordinates of the prism's top surface corners in the
     *                    image
     **/
    public double[] estimatePrismPose(MatOfPoint2f imagePoints) {

        // origin at the center of the top surface
        MatOfPoint3f objectPoints = new MatOfPoint3f(
                new Point3(-PRISM_LENGTH / 2, -PRISM_WIDTH / 2, 0), // top-left
                new Point3(PRISM_LENGTH / 2, -PRISM_WIDTH / 2, 0), // top-right
                new Point3(PRISM_LENGTH / 2, PRISM_WIDTH / 2, 0), // bottom-right
                new Point3(-PRISM_LENGTH / 2, PRISM_WIDTH / 2, 0) // bottom-left
        );

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        boolean success = Calib3d.solvePnP(objectPoints, imagePoints, CAMERA_INTRINSIC_MATRIX, DISTORTION_COEFFICIENTS,
                rvec, tvec);

        if (!success) {
            return new double[] { 0, 0, 0, 0 };
        }

        Mat rotationMatrix = new Mat();
        Calib3d.Rodrigues(rvec, rotationMatrix);

        double[] rotMat = new double[9];
        rotationMatrix.get(0, 0, rotMat);

        double rotationAngle = Math.atan2(rotMat[1], rotMat[0]) * 180 / Math.PI;

        double tiltRad = CAMERA_TILT_ANGLE * Math.PI / 180.0;
        double[][] cam2world = {
                { 1, 0, 0 },
                { 0, Math.cos(tiltRad), -Math.sin(tiltRad) },
                { 0, Math.sin(tiltRad), Math.cos(tiltRad) }
        };

        double[] translation = new double[3];
        tvec.get(0, 0, translation);

        double[] worldPos = new double[3];
        for (int i = 0; i < 3; i++) {
            worldPos[i] = 0;
            for (int j = 0; j < 3; j++) {
                worldPos[i] += cam2world[i][j] * translation[j];
            }
        }

        worldPos[2] = CAMERA_HEIGHT - worldPos[2];

        // [x, y, z, rotationAngle]
        return new double[] { worldPos[0], worldPos[1], worldPos[2], rotationAngle };
    }

    public void reset() {
        resultAvailable = false;
    }

    public void setAcceptedColors(boolean blue, boolean red, boolean yellow) {
        if (!available)
            return;
        String[] colors = new String[3];
        int i = 0;
        if (blue)
            colors[i++] = "blue";
        if (red)
            colors[i++] = "red";
        if (yellow)
            colors[i++] = "yellow";
        ConfigVariables.Camera.ACCEPTED_COLORS = Arrays.copyOf(colors, i);
    }

    public void setColor(String classname) {
        if (!available)
            return;
        switch (classname) {
            case "blue":
                limelight.updatePythonInputs(0, 0, 0, 0, 0, 0, 0, 0);
                break;
            case "red":
                limelight.updatePythonInputs(1, 0, 0, 0, 0, 0, 0, 0);
                break;
            case "yellow":
                limelight.updatePythonInputs(2, 0, 0, 0, 0, 0, 0, 0);
        }
    }

    public String getClassname() {
        if (!available || !resultAvailable)
            return "blue";
        return detectorResult.getClassName();
    }

    public double getTx() {
        if (!available || !resultAvailable)
            return 0;
        return result.getTx();
    }

    public double getTy() {
        if (!available || !resultAvailable)
            return 0;
        return result.getTy();
    }
    public double getPx() {
        if (!available || !resultAvailable)
            return 0;
        return detectorResult.getTargetXPixels();
    }

    public double getPy() {
        if (!available || !resultAvailable)
            return 0;
        return detectorResult.getTargetYPixels();
    }

    public double getTl(){ // target latency
        if (!available || !resultAvailable)
            return 0;
        return result.getTargetingLatency() + result.getParseLatency();
    }
    public double getDx() {
        if (!available || !resultAvailable)
            return 0;
        double dcx = detectorResult.getTargetXDegreesNoCrosshair();
        double tx = detectorResult.getTargetXDegrees();
        //Dx = sintx h / cos (tx-acx) / sin(90-acx)
        double angle = Math.toRadians(tx - dcx);
        double h = ConfigVariables.Camera.CAMERA_HEIGHT;
        // division by zero check
        if( Math.cos(angle)==0 || Math.sin(Math.toRadians(90 - dcx))==0) return 0;
        double dx = Math.sin(Math.toRadians(tx)) * h / Math.cos(angle) / Math.sin(Math.toRadians(90 - dcx));
        return dx;
    }
    public double getDy(){
        if (!available || !resultAvailable)
            return 0;
        double dcy = detectorResult.getTargetYDegreesNoCrosshair();
        double ty = detectorResult.getTargetYDegrees();
        // Dy = sinty*h/cos(90-tilt-ty+acy)/sin(90-tilt-ty+acy-ty)
        double angle = Math.toRadians(90 - ConfigVariables.Camera.TILT_ANGLE - ty + dcy);
        double h = ConfigVariables.Camera.CAMERA_HEIGHT;
        if( Math.cos(angle)==0|| Math.sin(Math.toRadians(90 - ConfigVariables.Camera.TILT_ANGLE - ty + dcy - ty))==0) return 0;
        double dy = Math.sin(Math.toRadians(ty)) * h / Math.cos(angle) / Math.sin(Math.toRadians(90 - ConfigVariables.Camera.TILT_ANGLE - ty + dcy - ty));
        return dy;
    }
    public double getProportion() {
        if (!available || !resultAvailable)
            return 0;
        outerCorners = detectorResult.getTargetCorners();
        if (outerCorners == null || outerCorners.size() != 4) {
            resultAvailable = false;
            return 0;
        }
        double A = outerCorners.get(1).get(0) - outerCorners.get(0).get(0);
        double B = outerCorners.get(2).get(1) - outerCorners.get(1).get(1);
        return A/B;
    }
    public double getWorldx() {
        return poseResult[0];
    }

    public double getWorldy() {
        return -poseResult[1];
    }

    public double getAngle() {
        return poseResult[3]; // degree
    }

    /**
     * Update telemetry with camera status and detection information
     */
    public void updateTelemetry(TelemetryPacket packet) {
        // Basic camera status
        packet.put("limelight/available", available);
        packet.put("limelight/resultAvailable", resultAvailable);

        if (!available) {
            packet.put("limelight/status", "Camera not available");
            return;
        }

        // Result validity
        if (result != null) {
            packet.put("limelight/resultValid", result.isValid());
        } else {
            packet.put("limelight/resultValid", false);
        }

        // Detection results
        if (detectorResults != null) {
            packet.put("limelight/detectionsCount", detectorResults.size());

            // Show all detected classes
            if (!detectorResults.isEmpty()) {
                StringBuilder detectedClasses = new StringBuilder();
                for (int i = 0; i < detectorResults.size(); i++) {
                    if (i > 0)
                        detectedClasses.append(", ");
                    detectedClasses.append(detectorResults.get(i).getClassName());
                }
                packet.put("limelight/detectedClasses", detectedClasses.toString());
            }
        } else {
            packet.put("limelight/detectionsCount", 0);
        }

        // Accepted colors configuration
        if (ConfigVariables.Camera.ACCEPTED_COLORS != null) {
            packet.put("limelight/acceptedColors", Arrays.toString(ConfigVariables.Camera.ACCEPTED_COLORS));
        }

        // Current target information
        if (resultAvailable && detectorResult != null) {
            packet.put("limelight/targetClass", detectorResult.getClassName());
            packet.put("limelight/targetConfidence", detectorResult.getConfidence());
            packet.put("limelight/tx", getTx());
            packet.put("limelight/ty", getTy());

            // Target corners if available
            if (outerCorners != null && outerCorners.size() == 4) {
                packet.put("limelight/cornersDetected", true);
                packet.put("limelight/cornerCount", outerCorners.size());
            } else {
                packet.put("limelight/cornersDetected", false);
            }
        } else {
            packet.put("limelight/targetClass", "none");
            packet.put("limelight/tx", 0.0);
            packet.put("limelight/ty", 0.0);
            packet.put("limelight/cornersDetected", false);
        }

        // World position data
        packet.put("limelight/worldX", getWorldx());
        packet.put("limelight/worldY", getWorldy());
        packet.put("limelight/angle", getAngle());

        // Status summary for quick debugging
        String status;
        if (!available) {
            status = "UNAVAILABLE";
        } else if (!resultAvailable) {
            status = "NO_TARGET";
        } else {
            status = "TARGET_DETECTED";
        }
        packet.put("limelight/status", status);
    }

}
