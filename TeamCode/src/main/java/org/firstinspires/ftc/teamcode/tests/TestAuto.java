package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import java.util.Collections;
import java.util.Comparator;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Rect;

import java.lang.Math;
import java.util.List;

@Autonomous(group="Auto")
public class TestAuto extends LinearOpMode {
    
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    //private ColorSensor color;
    //static final double     PIE=3.1415926535897932384626;
    static final double     PI=3.14;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 18.88;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 116.0 * PI;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;
    static final double     TPS = 1 * (175/ 60) * COUNTS_PER_WHEEL_REV;
    
    List<ColorBlobLocatorProcessor.Blob> possibleSample;
    
    Servo swervo;
    
    HuskyLens huskyLens;
    
    AprilTagProcessor tagProcessor;
    AprilTagDetection tag;
    
    VisionPortal tagPortal;
    VisionPortal samplePortal;
    
    ColorBlobLocatorProcessor colorLocatorBLUE;
    ColorBlobLocatorProcessor colorLocatorYELLOW;
    
    int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
    int portal1ViewId = viewIds[0];
    int portal2ViewId = viewIds[1];
    
    public int distance(int x, int y){ return (int) Math.sqrt((x-320)*(x-320)+(x-240)*(x-240)); }
    public double INCHtoCM(double INCH){ return INCH*2.54; }
    public void pos(double posNUM){ swervo.setPosition(0.05+posNUM*0.05); } // 1<= pos <=16
    public double angleRANGE(double angle){ return (-angle/12)+16; }
    
    public double sector(double width, double height , double angle){
        if(height > width) return 0;
        else return 90; 
    }
    public void cameraStart(){
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
    
    public void moveY(double distance){
        initialize();
        double part=distance*COUNTS_PER_MM*10;
        frontRightMotor.setTargetPosition((int)part);
        backLeftMotor.setTargetPosition((int)part);
        frontLeftMotor.setTargetPosition((int)part);
        backRightMotor.setTargetPosition((int)part);
        
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setVelocity(TPS);
        frontRightMotor.setVelocity(TPS);
        backLeftMotor.setVelocity(TPS);
        backRightMotor.setVelocity(TPS);
        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){}
        
    }
    
    public void moveX(double distance){
        initialize();
        double part=distance*COUNTS_PER_MM*10;
        frontRightMotor.setTargetPosition((int)-part);
        backLeftMotor.setTargetPosition((int)-part);
        frontLeftMotor.setTargetPosition((int)part);
        backRightMotor.setTargetPosition((int)part);
        
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setVelocity(TPS);
        frontRightMotor.setVelocity(TPS);
        backLeftMotor.setVelocity(TPS);
        backRightMotor.setVelocity(TPS);
        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){}
        
    }
    
    public void move(double rCM, double theta){
        double distanceX=Math.cos(theta)*rCM;
        double distanceY=Math.sin(theta)*rCM;
        moveX(distanceX);
        moveY(distanceY);
    }
    
    public void spin(double theta){
        initialize();
        double r=37.0;
        double part=(r*theta*COUNTS_PER_MM*10)*(15/9);
        frontRightMotor.setTargetPosition((int)-part);
        backLeftMotor.setTargetPosition((int)part);
        frontLeftMotor.setTargetPosition((int)part);
        backRightMotor.setTargetPosition((int)-part);
        
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setVelocity(TPS);
        frontRightMotor.setVelocity(TPS);
        backLeftMotor.setVelocity(TPS);
        backRightMotor.setVelocity(TPS);
        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){}
        
    }
    
    public void initialize(){
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        //swervo = hardwareMap.get(Servo.class, "swervo");
        
        // Wait for the game to start (driver presses PLAY)
        cameraStart();
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        
        //double testDistance=100.0;
        //double testAngle=3*PI/4;
        //move(testDistance,testAngle);
        //spin(PI);
        
        while (!isStopRequested() && opModeIsActive()) {
            /*
            if(tagProcessor.getDetections().size() > 0){
                /*
                for(int i=0;i<tagProcessor.getDetections().size();i++){
                    AprilTagDetection tmp = tagProcessor.getDetections().get(i);
                    telemetry.addData("tag","%d",tmp.id);
                }
                
                telemetry.addData("tag detected","%d", tagProcessor.getDetections().size());
                if(tagProcessor.getDetections().size() !=0) tag = tagProcessor.getDetections().get(0);
                
                if(tag.id>=11 && tag.id<=16){
                    telemetry.addData("x", "(%.2f)", INCHtoCM(tag.ftcPose.x));
                    telemetry.addData("y", "(%.2f)", INCHtoCM(tag.ftcPose.y));
                    telemetry.addData("z", "(%.2f)", INCHtoCM(tag.ftcPose.z));
                    telemetry.addData("roll", "(%.2f)", tag.ftcPose.roll);
                    telemetry.addData("pitch", "(%.2f)", tag.ftcPose.pitch);
                    telemetry.addData("yaw", "(%.2f)", tag.ftcPose.yaw);
                    telemetry.addData("bearing", "(%.2f)", tag.ftcPose.bearing);
                    telemetry.addData("range INCH", "(%.2f)", tag.ftcPose.range);
                    telemetry.addData("range CM", "(%.2f)", INCHtoCM(tag.ftcPose.range));
                }
                
            }
            */
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
            
            
            telemetry.addLine("--------------BLUE--------------");
            org.opencv.core.Size myBoxFitSize;
            
            if(possibleSample.size()!=0) {
                ColorBlobLocatorProcessor.Blob b = possibleSample.get(0);


                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                Rect myHorizontalBoxFit = boxFit.boundingRect();
                Point[] myBoxCorners = new Point[4];
                boxFit.points(myBoxCorners);

                for (int i = 0; i < 4; i++) {
                    telemetry.addLine(String.format("boxFit corner %d (%d,%d)", i, (int) myBoxCorners[i].x, (int) myBoxCorners[i].y));
                }
                telemetry.addData("angle", boxFit.angle);
                telemetry.addData("width", myBoxFitSize.width);
                telemetry.addData("height", myBoxFitSize.height);
                
                
                double clawANGLE = boxFit.angle+sector(myBoxFitSize.width, myBoxFitSize.height, boxFit.angle);
                telemetry.addData("claw", clawANGLE);
                telemetry.addData("pos","%.5f" ,angleRANGE(clawANGLE));
                //pos(angleRANGE(clawANGLE));
                
            }
            /*
            telemetry.addLine("Ratio");
            for(ColorBlobLocatorProcessor.Blob b : possibleSample){
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                Rect myHorizontalBoxFit = boxFit.boundingRect();
                
                telemetry.addLine(String.format("%4.2f", 
                                            b.getDensity()
                                            ));
                                            
                telemetry.addLine(String.format("%4.2f", 
                                            b.getAspectRatio()
                                            ));
            }
            */
            /*
            for(ColorBlobLocatorProcessor.Blob b : blobsBLUE)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                Rect myHorizontalBoxFit = boxFit.boundingRect();
                telemetry.addLine("X      Y      Width    Height    Area    Angle    Ratio");
                telemetry.addLine(String.format("%d    %d    %d    %d    %d    %.4f     %4.2f", 
                                            myHorizontalBoxFit.x, 
                                            myHorizontalBoxFit.y, 
                                            myHorizontalBoxFit.width, 
                                            myHorizontalBoxFit.height, 
                                            myHorizontalBoxFit.width*myHorizontalBoxFit.height,
                                            boxFit.angle,
                                            b.getDensity()
                                            ));
            }
            */
            /*
            telemetry.addLine("--------------YELLOW--------------");
            for(ColorBlobLocatorProcessor.Blob b : blobsYELLOW)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                Rect myHorizontalBoxFit = boxFit.boundingRect();
                telemetry.addLine("X      Y      Width    Height    Area    Angle    Ratio");
                telemetry.addLine(String.format("%d    %d    %d    %d    %d    %.4f     %4.2f", 
                                            myHorizontalBoxFit.x, 
                                            myHorizontalBoxFit.y, 
                                            myHorizontalBoxFit.width, 
                                            myHorizontalBoxFit.height, 
                                            myHorizontalBoxFit.width*myHorizontalBoxFit.height,
                                            boxFit.angle,
                                            b.getDensity()
                                            ));
            }
            */
            telemetry.update();
        }
    }
}