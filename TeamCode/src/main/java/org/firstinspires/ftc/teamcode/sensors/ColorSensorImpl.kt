package org.firstinspires.ftc.teamcode.sensors

import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables

class ColorSensorImpl(hardwareMap: HardwareMap) {
    var sensorColor: ColorSensor? = null
    var sensorDistance: DistanceSensor? = null

    init {
        sensorColor = hardwareMap.get(ColorSensor::class.java, "sensor_color_distance")
        sensorDistance = hardwareMap.get(DistanceSensor::class.java, "sensor_color_distance")
    }

    val distance: Double
        get() = sensorDistance?.getDistance(DistanceUnit.CM)?.toDouble() ?: Double.MAX_VALUE
    val red: Int
        get() = sensorColor?.red() ?: 0
    val green: Int
        get() = sensorColor?.green() ?: 0
    val blue: Int
        get() = sensorColor?.blue() ?: 0

    fun matchColor(): kotlin.String {
        // convert rgb values to hsv
        val hsvValues = FloatArray(3)
        Color.RGBToHSV(red, green, blue, hsvValues)

        // if too dark
        if (red + green + blue < 20) {
            return "unknown"
        }

        val hue = hsvValues[0]
        // Determine color based on hue range
        return when {
            hue in 180f..260f -> "blue"
            hue < 30f || hue > 330f -> "red"
            hue in 40f..90f -> "yellow"
            else -> "unknown"
        }
    }

    fun catched(): Boolean {
        // distance < 2cm
        return distance < 2
    }

    fun canTransfer(): Boolean {
        return matchColor() in ConfigVariables.Camera.ACCEPTED_COLORS && catched()
    }
}