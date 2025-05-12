package org.firstinspires.ftc.teamcode.commands.vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.slides.LowerSlide;
import org.firstinspires.ftc.teamcode.sensors.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

/**
 * Command to perform vision-assisted adjustments using Limelight
 */
public class VisionAdjustCommand extends CommandBase {
        private final LowerSlide lowSlide;
        private final Limelight camera;
        private final PIDFController pidY;
        private boolean isAdjusted = false;
        private boolean isAngleAdjusted = false;
        private double angleAccum = 0;
        private double angleNum = 1;

        public VisionAdjustCommand(LowerSlide lowSlide, Limelight camera) {
                this.lowSlide = lowSlide;
                this.camera = camera;
                this.pidY = new PIDFController(
                                ConfigVariables.Camera.PID_KP,
                                ConfigVariables.Camera.PID_KI,
                                ConfigVariables.Camera.PID_KD,
                                ConfigVariables.Camera.PID_KF);
                addRequirement(lowSlide);
        }

        @Override
        public void initialize() {
                isAdjusted = false;
                isAngleAdjusted = false;
                angleAccum = 0;
                angleNum = 1;
                pidY.reset();

                if (!camera.updateDetectorResult()) {
                        isAdjusted = true; // Skip if no detection
                        return;
                }

                camera.switchtoPython();
                camera.setColor(camera.getClassname());
        }

        @Override
        public void execute(TelemetryPacket packet) {
                if (isAdjusted) {
                        if (!isAngleAdjusted) {
                                // Processing angle for spinclaw
                                double angle = camera.getAngle(); // -90 ~ 90
                                angle = angle + ConfigVariables.Camera.ANGLE_OFFSET;
                                angleAccum += angle;
                                angleNum += 1;
                                packet.put("visionAdjust/angle", angle);

                                if (angleNum >= 10) { // Average over 10 samples
                                        double averageAngle = angleAccum / angleNum;
                                        lowSlide.spinclawSetPositionDeg(averageAngle);
                                        isAngleAdjusted = true;
                                        camera.switchtoNeural();
                                }
                                return;
                        }
                        return;
                }

                // Adjust slide position using PID
                camera.updateDetectorResult();
                double dy = camera.getY();
                double yPower = pidY.calculate(-dy);
                lowSlide.setSlidePower(yPower);

                // Add telemetry
                packet.put("visionAdjust/yDifference", dy);
                packet.put("visionAdjust/yPower", yPower);

                // Check if we're close enough
                if (Math.abs(dy) < ConfigVariables.Camera.DISTANCE_THRESHOLD) {
                        lowSlide.posNow(); // Hold current position
                        isAdjusted = true;
                        lowSlide.pos_hover(); // Move to hover position
                }
        }

        @Override
        public boolean isFinished() {
                return isAdjusted && isAngleAdjusted;
        }

        @Override
        public void end(boolean interrupted) {
                if (interrupted) {
                        lowSlide.stop();
                }
                camera.reset();
        }
}
