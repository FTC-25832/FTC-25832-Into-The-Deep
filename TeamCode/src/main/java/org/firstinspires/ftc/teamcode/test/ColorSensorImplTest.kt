package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.sensors.ColorSensorImpl
import java.util.Locale

@TeleOp(name = "SensorImpl: REVColorDistance", group = "Test")
class ColorSensorImplTest : LinearOpMode() {
    val telemetry: Telemetry = FtcDashboard.getInstance().telemetry
    override fun runOpMode() {
        val sensorColor = ColorSensorImpl(hardwareMap)
        if (sensorColor.sensorColor == null) {
            telemetry.addData("Error", "Color sensor not found!")
            telemetry.update()
            return
        }
        if (sensorColor.sensorDistance == null) {
            telemetry.addData("Error", "Distance sensor not found!")
            telemetry.update()
            return
        } else
            waitForStart()

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            telemetry.addData(
                "Distance (cm)",
                String.format(Locale.CHINA, "%.02f", sensorColor.distance)
            )
            telemetry.addData("Color", sensorColor.matchColor())
            telemetry.addData("Catched", sensorColor.catched())
            telemetry.addData("Can Transfer", sensorColor.canTransfer())
            telemetry.addData("Red", sensorColor.red)
            telemetry.addData("Green", sensorColor.green)
            telemetry.addData("Blue", sensorColor.blue)
            telemetry.update()
        }


    }
}