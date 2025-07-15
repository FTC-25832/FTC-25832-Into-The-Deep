package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class ConditionalCommand extends CommandBase {
    private final BooleanSupplier condition;
    private final Command command;
    private boolean shouldExecute;
    private boolean isFinished;

    public ConditionalCommand(BooleanSupplier condition, Command command) {
        this.condition = condition;
        this.command = command;

        // Merge child command's requirements
        command.getRequirements().forEach((Consumer<SubsystemBase>) requirements ->
                this.addRequirement(requirements));
    }

    @Override
    public void initialize() {
        shouldExecute = condition.getAsBoolean();
        isFinished = !shouldExecute;

        if (shouldExecute) {
            command.initialize();
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (!shouldExecute || isFinished) {
            return;
        }

        command.execute(packet);

        if (command.isFinished()) {
            command.end(false);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        if (shouldExecute && command != null) {
            command.end(interrupted);
        }
    }

    @Override
    public long getTimeout() {
        return shouldExecute ? command.getTimeout() : 0;
    }
}