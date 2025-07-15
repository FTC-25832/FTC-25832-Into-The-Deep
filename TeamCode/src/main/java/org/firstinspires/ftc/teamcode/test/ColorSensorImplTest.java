package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorImpl;

import java.util.Locale;

@TeleOp(name = "SensorImpl: REVColorDistance", group = "Test")
public class ColorSensorImplTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        ColorSensorImpl sensorColor = new ColorSensorImpl(hardwareMap);

        if (sensorColor.sensorColor == null) {
            telemetry.addData("Error", "Color sensor not found!");
            telemetry.update();
            return;
        }

        if (sensorColor.sensorDistance == null) {
            telemetry.addData("Error", "Distance sensor not found!");
            telemetry.update();
            return;
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance (cm)", String.format(Locale.CHINA, "%.02f", sensorColor.getDistance()));
            telemetry.addData("Color", sensorColor.matchColor());
            telemetry.addData("Catched", sensorColor.catched());
            telemetry.addData("Can Transfer", sensorColor.canTransfer());
            telemetry.addData("Red", sensorColor.getRed());
            telemetry.addData("Green", sensorColor.getGreen());
            telemetry.addData("Blue", sensorColor.getBlue());
            telemetry.update();
        }
    }
}
