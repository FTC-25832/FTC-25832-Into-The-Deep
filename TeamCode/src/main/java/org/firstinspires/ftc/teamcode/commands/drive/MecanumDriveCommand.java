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

                if(acceptManual){
                        PoseVelocity2d velocity = drive.updatePoseEstimate();
                        Pose2d currentPose = drive.localizer.getPose();
                         // Update pose estimate from drive
                        Vector2d input = new Vector2d(
                                -gamepad.left_stick_y * speedMultiplier,
                                -gamepad.left_stick_x * speedMultiplier
                        );

                        Vector2d driveVector;

                        // Check for field-centric mode
                        if (fieldCentric) {
                                // Use the robot's heading from the localizer for more accurate field-centric driving
                                double heading = currentPose.heading.toDouble();

                                // Rotate the input vector by the negative of the heading (field-centric)
                                double cosHeading = Math.cos(-heading);
                                double sinHeading = Math.sin(-heading);
                                driveVector = new Vector2d(
                                        input.x * cosHeading - input.y * sinHeading,
                                        input.x * sinHeading + input.y * cosHeading
                                );
                        } else {
                                // Robot-centric mode (no rotation of input vector)
                                driveVector = input;
                        }

                        // Pass the drive vector and rotation to the drive
                        drive.setDrivePowers(new PoseVelocity2d(
                                driveVector,
                                -gamepad.right_stick_x * ConfigVariables.General.DRIVE_ROTATE_FACTOR * speedMultiplier
                        ));
                        packet.put("X Position", currentPose.position.x);
                        packet.put("Y Position", currentPose.position.y);
                        packet.put("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
                        packet.put("X Velocity", velocity.linearVel.x);
                        packet.put("Y Velocity", velocity.linearVel.y);
                        packet.put("Angular Velocity", velocity.angVel);
                }
                // Add telemetry
                packet.put("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
                packet.put("Speed Multiplier", speedMultiplier);
        }
        public void enableControl(){
                acceptManual = true;
        }
        public void disableControl(){
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
                        0
                ));
        }

        @Override
        public Set<SubsystemBase> getRequirements() {
                return Set.of();
        }
}