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
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;

@TeleOp(name = "Reset State")
//@Disabled
public class ResetState extends LinearOpMode {


    @Override
    public void runOpMode()
    {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        telemetry.setMsTransmissionInterval(11);

        File file = new File("/sdcard/FIRST/robot_states/robot_state.txt");

        // Create directory if it doesn't exist
        file.getParentFile().mkdirs();
        telemetry.addData("deleted", file.delete());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

        }

    }
}
