package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.commands.base.Command;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Collections;
import java.util.Set;

/**
 * Command to set the drivetrain speed (maxWheelVel and maxProfileAccel) at
 * runtime.
 * This allows dynamic adjustment of robot speed during autonomous or teleop.
 */
public class SetDriveSpeedCommand implements Command {
        private final double speed;
        private boolean finished = false;

        public SetDriveSpeedCommand(double speed) {
                this.speed = speed;
        }

        @Override
        public void initialize() {
                // Set both maxWheelVel and maxProfileAccel to the given speed
                MecanumDrive.PARAMS.maxWheelVel = speed;
                MecanumDrive.PARAMS.maxProfileAccel = speed;
                finished = true;
        }

        @Override
        public void execute(TelemetryPacket packet) {
                // Nothing to do, runs instantly
        }

        @Override
        public boolean isFinished() {
                return finished;
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
