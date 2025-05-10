/*
Copyright 2024 FIRST Tech Challenge Team 25832

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class TestTageye extends LinearOpMode {
    int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

    @Override
    public void runOpMode() {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(viewIds[0])
                .setCamera(hardwareMap.get(WebcamName.class, "tageye"))
                .build();

        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (!isStopRequested() && opModeIsActive()) {
            if(tagProcessor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                if(tag.id>=11 && tag.id<=16){
                    telemetry.addData("x", "(%.2f)", tag.ftcPose.x);
                    telemetry.addData("y", "(%.2f)", tag.ftcPose.y);
                    telemetry.addData("z", "(%.2f)", tag.ftcPose.z);
                    telemetry.addData("roll", "(%.2f)", tag.ftcPose.roll);
                    telemetry.addData("pitch", "(%.2f)", tag.ftcPose.pitch);
                    telemetry.addData("yaw", "(%.2f)", tag.ftcPose.yaw);
                }
                
            }
            telemetry.update();
        }
    }
}
