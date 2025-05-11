package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.base.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

/**
 * teleop driving with field-centric control
 */
public class MecanumDriveCommand extends CommandBase {
        private final Drivetrain drive;
        private final Gamepad gamepad;
        private final IMU imu;

        /**
         * @param drive   The drive subsystem
         * @param gamepad The gamepad to read input from
         * @param imu     The IMU for field-centric control
         */
        public MecanumDriveCommand(Drivetrain drive, Gamepad gamepad, IMU imu) {
                this.drive = drive;
                this.gamepad = gamepad;
                this.imu = imu;
                addRequirement(drive);
        }

        @Override
        public void execute(TelemetryPacket packet) {
                double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad.left_stick_x;
                double rx = gamepad.right_stick_x;

                // Get heading from IMU for field-centric control
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Drive using field-centric controls
                drive.drive(x, y, rx, botHeading);

                packet.put("drive/input/x", x);
                packet.put("drive/input/y", y);
                packet.put("drive/input/rx", rx);
                packet.put("drive/heading", botHeading);
        }

        @Override
        public boolean isFinished() {
                // This is the default drive command, so it should never finish
                return false;
        }

        @Override
        public void end(boolean interrupted) {
                // Stop the drive motors when the command ends
                drive.stop();
        }
}
