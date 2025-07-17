package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.Arrays;

public class ColorSensorImpl {
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;

    public ColorSensorImpl(HardwareMap hardwareMap) {
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    public double getDistance() {
        return sensorDistance != null ? sensorDistance.getDistance(DistanceUnit.CM) : Double.MAX_VALUE;
    }

    public int getRed() {
        return sensorColor != null ? sensorColor.red() : 0;
    }

    public int getGreen() {
        return sensorColor != null ? sensorColor.green() : 0;
    }

    public int getBlue() {
        return sensorColor != null ? sensorColor.blue() : 0;
    }

    public String matchColor() {
        // convert rgb values to hsv
        float[] hsvValues = new float[3];
        Color.RGBToHSV(getRed(), getGreen(), getBlue(), hsvValues);

        // if too dark
        if (getRed() + getGreen() + getBlue() < 20) {
            return "unknown";
        }

        float hue = hsvValues[0];
        // Determine color based on hue range
        if (hue >= 180f && hue <= 260f) {
            return "blue";
        } else if (hue < 30f || hue > 330f) {
            return "red";
        } else if (hue >= 40f && hue <= 90f) {
            return "yellow";
        } else {
            return "unknown";
        }
    }

    public boolean caught() {
        // distance < 5cm
        return getDistance() < 5;
    }

    public boolean caughtDefaultTrue() {
        double distance = getDistance();
        // distance < 5cm
        return distance < 5 || distance >= Double.MAX_VALUE - 1;
    }

    public boolean canTransfer() {
        String color = matchColor();
        return Arrays.asList(ConfigVariables.Camera.ACCEPTED_COLORS).contains(color) && caught();
    }
}