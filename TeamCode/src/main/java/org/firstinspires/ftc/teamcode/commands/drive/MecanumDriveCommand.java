package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.control.ConfigVariables;

import java.util.Set;

public class MecanumDriveCommand implements Command {
        private final MecanumDrive drive;
        private final Gamepad gamepad;

        public MecanumDriveCommand(MecanumDrive drive, Gamepad gamepad) {
                this.drive = drive;
                this.gamepad = gamepad;
        }

        @Override
        public void initialize() {
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Read pose from localizer
                Pose2d currentPose = drive.localizer.getPose();

                // Create a vector from the gamepad x/y inputs
                Vector2d input = new Vector2d(-gamepad.left_stick_y, -gamepad.left_stick_x);

                // Rotate the input vector by the negative of the heading (field-centric)
                double heading = currentPose.heading.toDouble();
                double cosHeading = Math.cos(-heading);
                double sinHeading = Math.sin(-heading);
                Vector2d rotated = new Vector2d(
                                input.x * cosHeading - input.y * sinHeading,
                                input.x * sinHeading + input.y * cosHeading);

                // Pass in the rotated input + right stick value for rotation
                drive.setDrivePowers(new PoseVelocity2d(
                                rotated,
                                -gamepad.right_stick_x* ConfigVariables.General.DRIVE_ROTATE_FACTOR));

                // Update drive pose
                drive.updatePoseEstimate();

                // Add telemetry
                packet.put("x", currentPose.position.x);
                packet.put("y", currentPose.position.y);
                packet.put("heading", Math.toDegrees(currentPose.heading.toDouble()));
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
