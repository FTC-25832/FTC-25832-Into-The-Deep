package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class WaitForConditionCommand extends CommandBase {
    private final BooleanSupplier condition;
    private final long timeout;
    private final Command command;
    private long startTime;
    private boolean conditionMet;
    private boolean isFinished;
    private boolean commandStarted;

    public WaitForConditionCommand(BooleanSupplier condition, long timeout, Command command) {
        this.condition = condition;
        this.timeout = timeout;
        this.command = command;

        command.getRequirements().forEach((Consumer<SubsystemBase>) requirements ->
                this.addRequirement(requirements));
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        conditionMet = false;
        isFinished = false;
        commandStarted = false;
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (isFinished) {
            return;
        }

        if (command.getTimeout() > 0 && (System.currentTimeMillis() - startTime) >= command.getTimeout()) {
            isFinished = true;
            return;
        }

        if (!conditionMet) {
            conditionMet = condition.getAsBoolean();
            if (conditionMet) {
                command.initialize();
                commandStarted = true;
            }
            return;
        }

        if (commandStarted) {
            command.execute(packet);

            if (command.isFinished()) {
                command.end(false);
                isFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if (commandStarted) {
            command.end(interrupted);
        }
    }

    @Override
    public long getTimeout() {
        return timeout;
    }
}