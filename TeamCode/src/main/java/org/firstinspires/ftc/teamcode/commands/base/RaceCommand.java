package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.HashSet;
import java.util.Set;

public class RaceCommand extends CommandBase {
    private final Command[] commands;
    private boolean isFinished = false;

    public RaceCommand(Command... commands) {
        this.commands = commands;
    }

    @Override
    public void initialize() {
        isFinished = false;
        for (Command cmd : commands) {
            cmd.initialize();
            CommandScheduler.getInstance().schedule(cmd);
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        for (Command cmd : commands) {
            if (cmd.isFinished()) {
                isFinished = true;
                return;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            for (Command cmd : commands) {
                cmd.end(true);
            }
        }
    }
    @Override
    public long getTimeout() {
        return 0;
    }
    @Override
    public Set<SubsystemBase> getRequirements() {
        Set<SubsystemBase> requirements = new HashSet<>();
        for (Command cmd : commands) {
            requirements.addAll(cmd.getRequirements());
        }
        return requirements;
    }
}