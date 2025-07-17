package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.HashSet;
import java.util.Set;

public class WaitCommand implements Command {
    private final double seconds;
    private long startTime;

    public WaitCommand(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // Nothing to do while waiting
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= seconds * 1000;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up
    }

    @Override
    public Set<SubsystemBase> getRequirements() {
        return new HashSet<>();
    }
}
