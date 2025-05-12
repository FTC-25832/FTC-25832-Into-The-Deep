package org.firstinspires.ftc.teamcode.sensors.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {
    HardwareMap hardwareMap;
    LLResult result;
    public List<LLResultTypes.DetectorResult> detectorResults;
    public LLResultTypes.DetectorResult detectorResult;
    public boolean available = true;
    public boolean resultAvailable = false;
    public Limelight3A limelight;
    public void initialize(HardwareMap map){
        hardwareMap = map;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight == null){
            available = false;
            return;
        }
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
    }
    public void cameraStart(){
        limelight.start();
        limelight.reloadPipeline();
    }
    /*
     * return true if the result is valid (detected)
     */
    public boolean updateDetectorResult(){
        result = limelight.getLatestResult();
        if (result.isValid()){
            detectorResults = result.getDetectorResults();
            if (detectorResults.isEmpty()){
                resultAvailable = false;
                return false;
            }
            detectorResult = detectorResults.get(0);
            resultAvailable = true;
            return true;
        }
        resultAvailable = false;
        return false;
    }
    public void switchtoPython(){
        limelight.pipelineSwitch(1);
    }
    public void switchtoNeural(){
        limelight.pipelineSwitch(0);
    }
//    public double getAngle(){
//        if(!available ||!resultAvailable) return 0;
//        List<List<Double>> corners = detectorResult.getTargetCorners();
//        if (!corners.isEmpty()) {
//            double width = Math.sqrt(Math.pow(corners.get(1).get(0) - corners.get(0).get(0), 2) +
//                    Math.pow(corners.get(1).get(1) - corners.get(0).get(1), 2));
//            double height = Math.sqrt(Math.pow(corners.get(3).get(0) - corners.get(0).get(0), 2) +
//                    Math.pow(corners.get(3).get(1) - corners.get(0).get(1), 2));
//            return Math.atan2(height, width);
//        }
//        return 0;
//    }
    public void reset(){
        resultAvailable = false;
    }
    public void setColor(String classname){
        if(!available) return;
        switch(classname){
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
    public double getAngle(){
        if(!available) return 0;
        return limelight.getLatestResult().getPythonOutput()[1] * 5;
    }
    public String getClassname(){
        if(!available ||!resultAvailable) return "blue";
        return detectorResult.getClassName();
    }
    public double getX(){
        if(!available ||!resultAvailable) return 0;
        return result.getTx();
    }
    public double getY(){
        if(!available ||!resultAvailable) return -2;
        return result.getTy();
    }

}