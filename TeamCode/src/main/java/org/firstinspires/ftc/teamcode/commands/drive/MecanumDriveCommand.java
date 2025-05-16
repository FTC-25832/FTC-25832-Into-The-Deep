package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.Set;

public class MecanumDriveCommand implements Command {
        private final MecanumDrive drive;
        private final Gamepad gamepad;
        private final IMU imu;

        public MecanumDriveCommand(MecanumDrive drive, Gamepad gamepad, IMU imu) {
                this.drive = drive;
                this.gamepad = gamepad;
                this.imu = imu;
        }

        @Override
        public void initialize() {
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Create a vector from the gamepad x/y inputs
                Vector2d input = new Vector2d(-gamepad.left_stick_y, -gamepad.left_stick_x);

                // Rotate the input vector by the negative of the heading (field-centric)
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                // double heading = currentPose.heading.toDouble();
                double cosHeading = Math.cos(-heading);
                double sinHeading = Math.sin(-heading);
                Vector2d rotated = new Vector2d(
                                input.x * cosHeading - input.y * sinHeading,
                                input.x * sinHeading + input.y * cosHeading);

                // Pass in the rotated input + right stick value for rotation
                drive.setDrivePowers(new PoseVelocity2d(
                                rotated,
                                -gamepad.right_stick_x * ConfigVariables.General.DRIVE_ROTATE_FACTOR));

                // Add telemetry
                packet.put("heading", Math.toDegrees(heading));
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
                drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(0, 0),
                                0));
        }

        @Override
        public Set<SubsystemBase> getRequirements() {
                return Set.of(); // MecanumDrive is not a subsystem
        }
}
