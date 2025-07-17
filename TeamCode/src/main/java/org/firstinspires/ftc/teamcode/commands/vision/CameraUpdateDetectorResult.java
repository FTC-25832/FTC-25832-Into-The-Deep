package org.firstinspires.ftc.teamcode.commands.vision;

import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;

/**
 * Command to perform vision-assisted adjustments using Limelight
 */
public class CameraUpdateDetectorResult extends CommandBase {
    Limelight camera;

    public CameraUpdateDetectorResult(Limelight camera) { // Gamepad gamepad1
        this.camera = camera;
    }

    @Override
    public void initialize() {
        camera.updateDetectorResult();
    }

    // This command must be interrupted after 500ms to stop
    @Override
    public long getTimeout() {
        return 0;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
