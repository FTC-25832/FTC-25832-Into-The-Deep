package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class SequentialCommandGroup extends CommandBase {
    private final List<Command> commands;
    private int currentCommandIndex = -1;
    private Command currentCommand = null;
    private long groupStartTime;
    private long currentCommandStartTime;
    private boolean isFinished = false;
    public long timeout = 0; // 整个组的超时时间

    public SequentialCommandGroup(Command... commands) {
        this.commands = new ArrayList<>();
        addCommands(commands);
    }

    // 添加这个方法来支持添加多个命令
    public void addCommands(Command... commands) {
        for (Command cmd : commands) {
            this.commands.add(cmd);
            // 合并所有子命令的requirements
            cmd.getRequirements().forEach(this::addRequirement);
        }
    }

    @Override
    public void initialize() {
        currentCommandIndex = 0;
        isFinished = false;
        groupStartTime = System.currentTimeMillis();

        if (!commands.isEmpty()) {
            currentCommand = commands.get(0);
            currentCommandStartTime = System.currentTimeMillis();
            currentCommand.initialize();
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (isFinished || commands.isEmpty()) {
            return;
        }

        // Cache current time once per execute call
        long currentTime = System.currentTimeMillis();

        // Check group timeout using cached time
        if (timeout > 0 && (currentTime - groupStartTime) >= timeout) {
            cancelCurrentCommand(true);
            isFinished = true;
            return;
        }

        // Execute current command
        currentCommand.execute(packet);

        // Check if current command is finished
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            moveToNextCommand(currentTime);
        }
        // Check current command timeout using cached time
        else if (currentCommand.getTimeout() > 0 &&
                (currentTime - currentCommandStartTime) >= currentCommand.getTimeout()) {
            cancelCurrentCommand(true);
            moveToNextCommand(currentTime);
        }
    }

    private void moveToNextCommand(long currentTime) {
        currentCommandIndex++;
        if (currentCommandIndex < commands.size()) {
            currentCommand = commands.get(currentCommandIndex);
            currentCommandStartTime = currentTime;
            currentCommand.initialize();
        } else {
            isFinished = true;
            currentCommand = null;
        }
    }

    private void cancelCurrentCommand(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished || commands.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && currentCommand != null) {
            cancelCurrentCommand(true);
        }
        currentCommand = null;
    }

    @Override
    public long getTimeout() {
        return timeout;
    }
}