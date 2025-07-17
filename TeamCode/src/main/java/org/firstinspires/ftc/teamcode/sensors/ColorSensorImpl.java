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

    // PERFORMANCE OPTIMIZATION: Cache readings to avoid multiple I2C calls
    private double cachedDistance = Double.MAX_VALUE;
    private int cachedRed = 0;
    private int cachedGreen = 0;
    private int cachedBlue = 0;
    private String cachedColor = "unknown";
    private boolean cachedCatched = false;
    private boolean cachedCanTransfer = false;
    private long lastUpdateTime = 0;
    private static final long CACHE_TIMEOUT_MS = 50; // Update cache every 50ms max

    public ColorSensorImpl(HardwareMap hardwareMap) {
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    /**
     * PERFORMANCE OPTIMIZATION: Update all sensor readings at once
     * Call this from bulk read manager or once per loop
     */
    private void updateCache() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < CACHE_TIMEOUT_MS) {
            return; // Use cached values
        }

        // Update all values at once to minimize I2C calls
        cachedDistance = sensorDistance != null ? sensorDistance.getDistance(DistanceUnit.CM) : Double.MAX_VALUE;
        cachedRed = sensorColor != null ? sensorColor.red() : 0;
        cachedGreen = sensorColor != null ? sensorColor.green() : 0;
        cachedBlue = sensorColor != null ? sensorColor.blue() : 0;

        // Update derived values
        cachedColor = calculateColor();
        cachedCatched = cachedDistance < 5;
        cachedCanTransfer = Arrays.asList(ConfigVariables.Camera.ACCEPTED_COLORS).contains(cachedColor)
                && cachedCatched;

        lastUpdateTime = currentTime;
    }

    public double getDistance() {
        updateCache();
        return cachedDistance;
    }

    public int getRed() {
        updateCache();
        return cachedRed;
    }

    public int getGreen() {
        updateCache();
        return cachedGreen;
    }

    public int getBlue() {
        updateCache();
        return cachedBlue;
    }

    private String calculateColor() {
        // convert rgb values to hsv
        float[] hsvValues = new float[3];
        Color.RGBToHSV(cachedRed, cachedGreen, cachedBlue, hsvValues);

        // if too dark
        if (cachedRed + cachedGreen + cachedBlue < 20) {
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

    public String matchColor() {
        updateCache();
        return cachedColor;
    }

    public boolean catched() {
        updateCache();
        return cachedCatched;
    }

    public boolean canTransfer() {
        updateCache();
        return cachedCanTransfer;
    }
}
