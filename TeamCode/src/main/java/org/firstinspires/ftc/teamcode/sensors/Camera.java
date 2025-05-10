package org.firstinspires.ftc.teamcode.sensors;

import java.util.Collections;
import java.util.Comparator;
import java.lang.Math;
import java.util.List;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.RotatedRect;
import org.opencv.core.Rect;



public class Camera {
    public Servo swervo, servo;

    
    List<ColorBlobLocatorProcessor.Blob> possibleSample;
    
    AprilTagProcessor tagProcessor;
    
    VisionPortal tagPortal;
    VisionPortal samplePortal;
    
    ColorBlobLocatorProcessor colorLocatorBLUE;
    ColorBlobLocatorProcessor colorLocatorYELLOW;
    
    HardwareMap hardwareMap;
    
    int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
    int portal1ViewId = viewIds[0];
    int portal2ViewId = viewIds[1];
    public void cameraStart(HardwareMap map){
        hardwareMap = map;
        swervo = hardwareMap.get(Servo.class, "swervo");
        servo = hardwareMap.get(Servo.class, "servo");

        colorLocatorBLUE = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        
        colorLocatorYELLOW = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search central 1/4 of camera view
                //.setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        //Position cameraPosition = new Position(DistanceUnit.CM, 0, 0, 0, 0);
        //YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
        tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

        tagPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal1ViewId)
                .setCamera(hardwareMap.get(WebcamName.class, "tageye"))
                .build();
                
        samplePortal = new VisionPortal.Builder()
                .addProcessor(colorLocatorBLUE)
                .addProcessor(colorLocatorYELLOW)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal2ViewId)
                .setCamera(hardwareMap.get(WebcamName.class, "allseeingeye"))
                .build();
        
    }
    
    public void pos(double posNUM){ swervo.setPosition(0.05+posNUM*0.05); } // 1<= pos <=16
    
    public double angleRANGE(double angle){ return (-angle/12)+16; }
    
    public double sector(double width, double height , double angle){
        if(height > width) return 0;
        else return 90; 
    }
    
    public void clawTurn(){
        possibleSample = colorLocatorBLUE.getBlobs();
        //possibleSample = colorLocatorYELLOW.getBlobs();
        possibleSample.addAll(colorLocatorYELLOW.getBlobs());
            
        ColorBlobLocatorProcessor.Util.filterByArea(10000, 1000000, possibleSample);
        //ColorBlobLocatorProcessor.Util.filterByDensity(double, double, java.util.List);
        //ColorBlobLocatorProcessor.Util.filterByArea(10000, 1000000, blobsYELLOW);

        Collections.sort(possibleSample, new Comparator<ColorBlobLocatorProcessor.Blob>(){
                public int compare(ColorBlobLocatorProcessor.Blob s1, ColorBlobLocatorProcessor.Blob s2) {
                return distance((int) s1.getBoxFit().center.x, (int) s1.getBoxFit().center.y)-distance((int) s2.getBoxFit().center.x, (int) s2.getBoxFit().center.y);
            }
        });
            
        //telemetry.addLine("--------------BLUE--------------");
        org.opencv.core.Size myBoxFitSize;
            
        if(possibleSample.size()!=0) {
            ColorBlobLocatorProcessor.Blob b = possibleSample.get(0);
                
            RotatedRect boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            Rect myHorizontalBoxFit = boxFit.boundingRect();
                
            //telemetry.addData("angle", boxFit.angle);
            //telemetry.addData("width", myBoxFitSize.width);
            //telemetry.addData("height", myBoxFitSize.height);

                
            double clawANGLE = boxFit.angle+sector(myBoxFitSize.width, myBoxFitSize.height, boxFit.angle);
            //telemetry.addData("claw", clawANGLE);
            //telemetry.addData("pos","%.5f" ,angleRANGE(clawANGLE));
            pos(angleRANGE(clawANGLE));
            
        }
    }
    public int distance(int x, int y){ return (int) Math.sqrt((x-320)*(x-320)+(y-240)*(y-240)); /* distance to center */ }
}