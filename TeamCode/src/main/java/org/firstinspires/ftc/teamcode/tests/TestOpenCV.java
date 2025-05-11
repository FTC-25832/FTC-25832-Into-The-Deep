package org.firstinspires.ftc.teamcode.tests;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group="Test")
public class TestOpenCV extends OpMode{
    OpenCvWebcam eye = null;
    int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class,"allseeingeye");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        //telemetry.addData("ID0",cameraMonitorViewId);
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal tagPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(viewIds[0])
                .setCamera(hardwareMap.get(WebcamName.class, "tageye"))
                .build();
        eye = OpenCvCameraFactory.getInstance().createWebcam(webcamName,viewIds[1]);
        eye.setPipeline(new BluePixelPipeline());
        eye.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                eye.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop(){

    }
    class sampleDetector extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;

        double leftavgfin;
        double rightavgfin;

        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);
            Rect leftRect = new Rect(1, 1,319, 359);
            Rect rightRect = new Rect(320, 1,319, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            return(output);
        }
    }
    public class BluePixelPipeline extends OpenCvPipeline {

        private final Mat yCbCr = new Mat();
        private final Mat mask = new Mat();
        private final Mat output = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);

            // Define the range of blue in YCbCr (Adjust these values as needed)
            //Scalar lowerBlue = new Scalar(0, 100, 0);  // Adjust min Y, Cb, Cr values
            //Scalar upperBlue = new Scalar(255, 255, 140);  // Adjust max Y, Cb, Cr values
            Scalar lowerBlue = new Scalar(0, 0, 100);
            Scalar upperBlue = new Scalar(255, 120, 255);
            // Threshold the image to isolate blue
            Core.inRange(yCbCr, lowerBlue, upperBlue, mask);

            // Create an output image where all pixels are black
            output.setTo(new Scalar(0, 0, 0));

            // Copy only the blue pixels from the input to the output
            input.copyTo(output, mask);

            return output;
        }
    }
}
