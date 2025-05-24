package org.firstinspires.ftc.teamcode.commands.base;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.Arrays;
import java.util.HashSet;

public class RoadrunnerActionCommand extends CommandBase {

    private final Action roadrunnerAction;
    private boolean actionFinished = false;

    /**
     * Creates a new RoadrunnerActionCommand.
     * @param roadrunnerAction The Roadrunner Action to be executed.
     * @param requiredSubsystems The subsystems required by this action (e.g., DrivetrainSubsystem).
     */
    public RoadrunnerActionCommand(Action roadrunnerAction, SubsystemBase... requiredSubsystems) {
        if (roadrunnerAction == null) {
            throw new IllegalArgumentException("Roadrunner Action cannot be null");
        }
        this.roadrunnerAction = roadrunnerAction;
        if (requiredSubsystems != null) {
            addRequirements(requiredSubsystems); // Internally adds to the requirements set in CommandBase
        }
    }

    @Override
    public void initialize() {
        actionFinished = false;
        // Roadrunner actions are typically initialized by their first run() call if they need explicit init.
        // Or, if Roadrunner's Action interface had an init() method, we'd call it here.
        // For now, we assume run() handles its own initialization logic.
    }

    @Override
    public void execute(TelemetryPacket packet) {
        // The run() method of a Roadrunner Action returns true if it's still running,
        // and false if it has completed.
        if (!actionFinished) {
            boolean stillRunning = roadrunnerAction.run(packet);
            if (!stillRunning) {
                actionFinished = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return actionFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Roadrunner's Action interface does not have a specific "end" or "cancel" method.
        // If an action is interrupted, its execution simply stops.
        // If cleanup were needed, it would have to be part of the Action's run() logic
        // or a separate cancellation mechanism if Roadrunner provided one.
        // For now, if interrupted is true, we mainly rely on the command no longer being executed.
        actionFinished = true; // Ensure it's marked finished if interrupted.
    }
}
