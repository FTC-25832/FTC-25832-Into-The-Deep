package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.Map;

/**
 * Container for telemetry information with debug mode support.
 */
public class TelemetryPacket extends com.acmerobotics.dashboard.telemetry.TelemetryPacket {
    private static final Canvas DEFAULT_FIELD = new Canvas();

    static {
        DEFAULT_FIELD.setAlpha(0.4);
        DEFAULT_FIELD.drawImage("/dash/into-the-deep.png", 0, 0, 144, 144);
        DEFAULT_FIELD.setAlpha(1.0);
        DEFAULT_FIELD.drawGrid(0, 0, 144, 144, 7, 7);
    }


    /**
     * Creates a new telemetry packet.
     */
    public TelemetryPacket(boolean drawDefaultField) {
        super(drawDefaultField);
    }

    public TelemetryPacket() {
        this(true);
    }

    public void put(String key, Object value) {
        if (ConfigVariables.General.DEBUG_MODE) {
            super.put(key, value);
        }
    }

    public void putAll(Map<String, Object> map) {
        if (ConfigVariables.General.DEBUG_MODE) {
            super.putAll(map);
        }
    }

    public void addLine(String line) {
        if (ConfigVariables.General.DEBUG_MODE) {
            super.addLine(line);
        }
    }

    public void clearLines() {
        if (ConfigVariables.General.DEBUG_MODE) {
            super.clearLines();
        }
    }

}
