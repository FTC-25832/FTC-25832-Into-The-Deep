package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Array;

public class Limelight {
    HardwareMap hardwareMap;
    double[] data = new double[8];
    public Limelight3A limelight;
    public void initialize(HardwareMap map){
        hardwareMap = map;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
    }
    public void cameraStart(){
        limelight.start();
        limelight.reloadPipeline();
    }
    public void updatePythonOutput(){
        this.data = limelight.getLatestResult().getPythonOutput();
    }
    public double getAngle(){
        return data[1]*5;
    }
    public double getX(){
        return data[2];
    }
    public double getY(){
        return data[3];
    }

}