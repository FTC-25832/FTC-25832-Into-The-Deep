package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Arrays;
import java.util.Collections;
import java.util.Set;

/**
 * Command to set the drivetrain speed (maxWheelVel and maxProfileAccel) at
 * runtime.
 * This allows dynamic adjustment of robot speed during autonomous or teleop.
 */
public class SetDriveSpeedCommand implements Command {
        private final double speed;
        private final MecanumDrive drive;

        public SetDriveSpeedCommand(MecanumDrive drive, double speed) {
                this.drive = drive;
                this.speed = speed;
        }

        @Override
        public void initialize() {
                // Set both maxWheelVel and maxProfileAccel to the given speed
                drive.PARAMS.maxWheelVel = speed;
                drive.PARAMS.maxProfileAccel = speed;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Always report the current drive speed for dashboard and driver station
                // visibility
                packet.put("drivespeed/maxWheelVel", drive.PARAMS.maxWheelVel);
                packet.put("drivespeed/maxProfileAccel", drive.PARAMS.maxProfileAccel);
        }

        @Override
        public boolean isFinished() {
                return true;
        }

        @Override
        public void end(boolean interrupted) {
                // Nothing to clean up
        }
        @Override
        public Set<SubsystemBase> getRequirements() {
                return Collections.emptySet();
        }
}
