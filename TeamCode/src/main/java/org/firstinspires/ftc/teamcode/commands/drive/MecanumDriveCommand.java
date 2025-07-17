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

        // Speed multipliers for fine control
        private double speedMultiplier = 1.0;
        private boolean fieldCentric = true;
        private boolean acceptManual = true;

        public MecanumDriveCommand(MecanumDrive drive, Gamepad gamepad) {
                this.drive = drive;
                this.gamepad = gamepad;
        }

        @Override
        public void initialize() {
                // Nothing to initialize
        }

        @Override
        public void execute(TelemetryPacket packet) {

                if (acceptManual) {
                        PoseVelocity2d velocity = drive.updatePoseEstimate();
                        Pose2d currentPose = drive.localizer.getPose();

                        // Check if robot is in team box corner and apply slowdown
                        double currentSpeedMultiplier = speedMultiplier;
                        boolean inTeamBox = isInTeamBoxCorner(currentPose);
                        if (ConfigVariables.General.ENABLE_TEAMBOX_SLOWDOWN && inTeamBox) {
                                currentSpeedMultiplier *= ConfigVariables.General.TEAMBOX_SLOWDOWN_FACTOR;
                        }

                        // Update pose estimate from drive
                        Vector2d input = new Vector2d(
                                        -gamepad.left_stick_y * currentSpeedMultiplier,
                                        -gamepad.left_stick_x * currentSpeedMultiplier);

                        Vector2d driveVector;

                        // Check for field-centric mode
                        if (fieldCentric) {
                                // Use the robot's heading from the localizer for more accurate field-centric
                                // driving
                                double heading = currentPose.heading.toDouble();

                                // Rotate the input vector by the negative of the heading (field-centric)
                                double cosHeading = Math.cos(-heading);
                                double sinHeading = Math.sin(-heading);
                                driveVector = new Vector2d(
                                                input.x * cosHeading - input.y * sinHeading,
                                                input.x * sinHeading + input.y * cosHeading);
                        } else {
                                // Robot-centric mode (no rotation of input vector)
                                driveVector = input;
                        }

                        // Pass the drive vector and rotation to the drive
                        drive.setDrivePowers(new PoseVelocity2d(
                                        driveVector,
                                        -gamepad.right_stick_x * ConfigVariables.General.DRIVE_ROTATE_FACTOR
                                                        * currentSpeedMultiplier));
                        packet.put("X Position", currentPose.position.x);
                        packet.put("Y Position", currentPose.position.y);
                        packet.put("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
                        packet.put("X Velocity", velocity.linearVel.x);
                        packet.put("Y Velocity", velocity.linearVel.y);
                        packet.put("Angular Velocity", velocity.angVel);
                        packet.put("In Team Box", inTeamBox);
                        packet.put("Current Speed Multiplier", currentSpeedMultiplier);
                }
                // Add telemetry
                packet.put("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
                packet.put("Speed Multiplier", speedMultiplier);
        }

        /**
         * Checks if the robot is currently in a team box corner
         * Assumes field coordinate system with (0,0) at center
         * Field is approximately 144" x 144" (12 feet square)
         * Team boxes are in the four corners of the field
         * 
         * @param pose Current robot pose from localizer
         * @return true if robot is in a team box corner
         */
        private boolean isInTeamBoxCorner(Pose2d pose) {
                if (!ConfigVariables.General.ENABLE_TEAMBOX_SLOWDOWN) {
                        return false;
                }

                double x = pose.position.x;
                double y = pose.position.y;
                double cornerSize = ConfigVariables.General.TEAMBOX_CORNER_SIZE;

                // Field dimensions (assuming 144" x 144" field with center at 0,0)
                double fieldHalfSize = 72.0; // Half of 144 inches

                // Check if robot is in any of the four corners
                boolean inTopRight = (x > fieldHalfSize - cornerSize) && (y > fieldHalfSize - cornerSize);
                boolean inTopLeft = (x < -(fieldHalfSize - cornerSize)) && (y > fieldHalfSize - cornerSize);
                boolean inBottomRight = (x > fieldHalfSize - cornerSize) && (y < -(fieldHalfSize - cornerSize));
                boolean inBottomLeft = (x < -(fieldHalfSize - cornerSize)) && (y < -(fieldHalfSize - cornerSize));

                return inTopRight || inTopLeft || inBottomRight || inBottomLeft;
        }

        public void enableControl() {
                acceptManual = true;
        }

        public void disableControl() {
                acceptManual = false;
        }

        @Override
        public boolean isFinished() {
                return false;
        }

        @Override
        public void end(boolean interrupted) {
                // Stop the drive when the command ends
                drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(0, 0),
                                0));
        }

        @Override
        public Set<SubsystemBase> getRequirements() {
                return Set.of();
        }
}
