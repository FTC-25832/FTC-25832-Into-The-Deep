package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive; // Your existing MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

    public final MecanumDrive drive; // Public final to allow access from commands if necessary

    /**
     * Creates a new DrivetrainSubsystem.
     * @param hardwareMap The hardware map.
     * @param initialPose The initial pose of the robot.
     */
    public DrivetrainSubsystem(HardwareMap hardwareMap, Pose2d initialPose) {
        super("Drivetrain"); // Call SubsystemBase constructor with name
        this.drive = new MecanumDrive(hardwareMap, initialPose);
        // No default command set here, as it's often context-dependent (TeleOp vs Auto)
    }

    @Override
    public void initialize(HardwareMap hardwareMap) {
        // The MecanumDrive itself is initialized in the constructor.
        // If there were other specific initializations for the subsystem wrapper,
        // they would go here.
    }

    @Override
    public void periodic(TelemetryPacket packet) {
        // MecanumDrive likely has its own update mechanism if it's doing localization
        // or other periodic tasks. If MecanumDrive.update() needs to be called,
        // this could be a place for it, though Roadrunner's actions often drive updates.
        // For now, we can leave this empty unless MecanumDrive requires explicit periodic calls
        // outside of action execution or TeleOp command loops.
        
        // Example if localizer updates were needed:
        // drive.updatePoseEstimate(); // Or whatever method Roadrunner uses
        // packet.put("Drivetrain/X", drive.pose.position.x);
        // packet.put("Drivetrain/Y", drive.pose.position.y);
        // packet.put("Drivetrain/Heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
    }

    @Override
    public void stop() {
        // If there's anything specific to shut down on the drivetrain when an OpMode stops,
        // like explicitly stopping motors (though this is often handled by the SDK).
        // For MecanumDrive, it doesn't have an explicit stop() method, motors will stop
        // when power is cut or no commands are sent.
        drive.setDrivePowers(0,0,0,0); // Example of stopping motors
    }

    // Convenience methods can be added here if needed, e.g.,
    // public void setDrivePowers(...) { drive.setDrivePowers(...); }
    // public Pose2d getPose() { return drive.pose; }
}
