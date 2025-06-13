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
import org.firstinspires.ftc.teamcode.sensors.limelight.LimeLightImageTools;


/**
 * This OpMode illustrates how to use the LimelightImageTools
 */
@TeleOp(name = "LL Tools Test", group = "Sensor")
//@Disabled
public class LimeLightToolsTest extends LinearOpMode {


    @Override
    public void runOpMode()
    {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        limelight.deleteSnapshots();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();


        LimeLightImageTools llIt = new LimeLightImageTools(limelight);  // instantiate the lime light tools class, pulls IP address from LimeLight
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


        int frames = 0;
        int droppedFrames = 0;
        boolean showProcessedImage = true;
        boolean lastAButton = false; // Store the state of the A button from the previous cycle
        long startTime = System.nanoTime();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastAButton) {
                showProcessedImage = !showProcessedImage;
                frames = 0;
                droppedFrames = 0;
                startTime = System.nanoTime();
            }
            lastAButton = gamepad1.a;

            Bitmap bmp;
            if (showProcessedImage) {
                bmp = llIt.getProcessedBMP();
            } else {
                bmp = llIt.getRawBMP();
            }

            if (bmp != null) {
                FtcDashboard.getInstance().sendImage(bmp);
                frames++;
            } else {
                droppedFrames++;
            }
            long currentTime = System.nanoTime();
            double elapsedTimeSeconds = (currentTime - startTime) / 1_000_000_000.0;
            double frameRate =  (double)frames/elapsedTimeSeconds;

            RobotLog.d("LL_Test  " + (showProcessedImage ? "Processed":"Raw") + "  good frames = " + frames +"   Dropped frame = "+ droppedFrames + " Frame/second="+ frameRate);
        }

    }
}
