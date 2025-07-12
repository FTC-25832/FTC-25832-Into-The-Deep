/*
Disclaimer - intended for development only.  Not recommended for competition or official events
Using the port forwarding for configuration or streaming to dashboard may use significant
bandwidth and cause problems with robots in a dense competition setting.

We do think that the setDriverStationStreamSource() method utilizes minimal bandwidth, sending
single snapshots as intended by the drivers station app and FTC SDK. Its use may be appropriate
at event venues.

Following is from any email with Danny Diaz, FTC Senior Engineering Manager
"With that in mind, we do have an important request regarding the use of this particular tool at
official FTC events. Due to the very limited and critical nature of Wi-Fi bandwidth at our events,
tools that place a significant burden on the network can unfortunately impact the overall event
experience for everyone. Therefore, we would greatly appreciate it if you could prominently
include a statement in your repository explicitly advising users that this tool should absolutely
not be used at official FTC events. This includes the competition area, pit area, and practice
fields. It's crucial to emphasize that using such a tool at an event is forbidden due to the
potential strain on the event's network infrastructure. This is a policy we maintain across the
board, and it's the same reason we don't allow streaming to the Driver Station."
*/
package org.firstinspires.ftc.teamcode.test;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
@TeleOp(name = "LL Pose Test", group = "Sensor")
//@Disabled
public class LimelightPoseDetectionTest extends LinearOpMode {

    Limelight limelight = new Limelight();
    @Override
    public void runOpMode()
    {
        limelight.initialize(hardwareMap);
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        limelight.cameraStart();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();


        LimeLightImageTools llIt = new LimeLightImageTools(limelight.limelight);  // instantiate the lime light tools class, pulls IP address from LimeLight
        //LimeLightImageTools llIt = new LimeLightImageTools("172.29.0.1"); // alternate constructor that provides the IP address

        // Set the LimeLight to provide the image source for the drivers station
        // "camera stream" during Init.  Great for quick verification of camera or for match setup
        // This probably is legal in matches, but should be verified by users.
        llIt.setDriverStationStreamSource();

        // Start port forwarding to allow accessing the limelight from external devices such as
        // a desktop/laptop, a phone or even the driver station (use browser already on DS) that is
        // connected to the Robot's WIFI network.  Point your favorite browser at:
        // http://192.168.43.1:5800/  - displays raw image before processing
        // http://192.168.43.1:5801/  - opens the limelight configuration tool!
        // http://192.168.43.1:5802/  - display the processed
        // This is NOT legal in competition?
        llIt.forwardAll();

        // If using Ftc Dash Board, this will stream the limelight camera to dashboard
        // dashboard is not legal in competition and discourages at events due to bandwidth
        FtcDashboard.getInstance().startCameraStream(llIt.getStreamSource(),10);

        waitForStart();


        /*  *** below here shows how to access a single image and send it to dashboard *** */
        // not really needed if using the startCameraStream() shown above, but might be useful
        // for capturing images at a particular time.


        while (opModeIsActive()) {
           limelight.updateDetectorResult();
           limelight.updatePosition();
           if(limelight.resultAvailable&&limelight.available){
               telemetry.addData("Corners", limelight.outerCorners);
               telemetry.addData("X", limelight.getWorldx());
               telemetry.addData("Y", limelight.getWorldy());
               telemetry.addData("Angle", limelight.getAngle());
               telemetry.addData("TX", limelight.getTx());
               telemetry.addData("TY", limelight.getTy());
               telemetry.addData("DX", limelight.getDx());
               telemetry.addData("DY", limelight.getDy());
               telemetry.addData("PX", limelight.getPx());
               telemetry.addData("PY", limelight.getPy());
               telemetry.addData("Proportion", limelight.getProportion());
               telemetry.update();
           }
            // Get the latest image from the limelight
            Bitmap bitmap = llIt.getProcessedBMP();

            // If the bitmap is not null, send it to the dashboard
            if (bitmap != null) {
                FtcDashboard.getInstance().sendImage(bitmap);
            } else {
                RobotLog.w("LimelightPoseDetectionTest", "Bitmap is null");
            }

            // Sleep for a short time to avoid overwhelming the dashboard
            sleep(100);
        }

    }
}
