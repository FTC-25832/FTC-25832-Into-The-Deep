package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class FunctionalCommand extends CommandBase {

    private final Runnable onInit;
    private final Runnable onExecute;
    private final BooleanSupplier isCommandFinished;
    private final Consumer<Boolean> onEnd;
    // Requirements are handled by CommandBase's internal set.

    /**
     * Primary constructor for a command defined by functional interfaces.
     *
     * @param onInit             Action to run upon initialization. Can be null.
     * @param onExecute          Action to run repeatedly while the command is scheduled. Can be null.
     * @param isCommandFinished  Supplier to determine if the command has finished. Can be null (defaults to true).
     * @param onEnd              Action to run when the command ends or is interrupted. Can be null.
     * @param requirements       Subsystems required by this command.
     */
    public FunctionalCommand(
            Runnable onInit,
            Runnable onExecute,
            BooleanSupplier isCommandFinished,
            Consumer<Boolean> onEnd,
            SubsystemBase... requirements
    ) {
        this.onInit = onInit;
        this.onExecute = onExecute;
        this.isCommandFinished = isCommandFinished;
        this.onEnd = onEnd;
        if (requirements != null) {
            addRequirements(requirements);
        }
    }

    // Convenience constructor for an "instant" command (runs once on init)
    public FunctionalCommand(Runnable onInit, SubsystemBase... requirements) {
        this(onInit, null, () -> true, null, requirements);
    }

    // Convenience constructor for a command that runs an execute action continuously (until interrupted)
    // and never finishes on its own.
    public FunctionalCommand(Runnable onExecute, SubsystemBase... requirements) {
        this(null, onExecute, () -> false, null, requirements);
    }
    
    // Convenience constructor for a command with init and execute, finishes when isCommandFinished is true.
    public FunctionalCommand(Runnable onInit, Runnable onExecute, BooleanSupplier isCommandFinished, SubsystemBase... requirements) {
        this(onInit, onExecute, isCommandFinished, null, requirements);
    }

    // Convenience constructor for a command with init, execute, and end. Finishes when isCommandFinished is true.
    public FunctionalCommand(Runnable onInit, Runnable onExecute, BooleanSupplier isCommandFinished, Consumer<Boolean> onEndAction, SubsystemBase... requirements) {
        this(onInit, onExecute, isCommandFinished, onEndAction, requirements);
    }


    @Override
    public void initialize() {
        if (onInit != null) {
            onInit.run();
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        if (onExecute != null) {
            onExecute.run();
        }
    }

    @Override
    public boolean isFinished() {
        if (isCommandFinished != null) {
            return isCommandFinished.getAsBoolean();
        }
        return true; // Default to finishing immediately if no condition is specified
    }

    @Override
    public void end(boolean interrupted) {
        if (onEnd != null) {
            onEnd.accept(interrupted);
        }
    }
}
