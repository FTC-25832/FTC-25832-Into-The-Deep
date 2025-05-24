package org.firstinspires.ftc.teamcode.commands.base;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class ParallelCommandGroup extends CommandGroupBase {

    private final List<Command> commands = new ArrayList<>();
    private final Set<Command> runningCommands = new HashSet<>();
    private boolean runWhenDisabled = false; // Typically false for parallel groups

    /**
     * Creates a new ParallelCommandGroup.
     * The group will run commands in parallel and end when all commands have finished.
     * @param commands The commands to run in parallel.
     */
    public ParallelCommandGroup(Command... commands) {
        this.commands.addAll(Arrays.asList(commands));
        // Collect all requirements from child commands
        for (Command cmd : this.commands) {
            // Subsystem requirements are managed by CommandBase's 'requirements' set.
            // addRequirement is inherited and adds to this set.
            cmd.getRequirements().forEach(this::addRequirement);
        }
    }

    @Override
    public void initialize() {
        runningCommands.clear();
        for (Command command : commands) {
            command.initialize();
            runningCommands.add(command);
        }
    }

    @Override
    public void execute(TelemetryPacket packet) {
        Set<Command> finishedThisCycle = new HashSet<>();
        for (Command command : runningCommands) {
            command.execute(packet);
            if (command.isFinished()) {
                command.end(false);
                finishedThisCycle.add(command);
            }
        }
        runningCommands.removeAll(finishedThisCycle);
    }

    @Override
    public boolean isFinished() {
        return runningCommands.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        // If the group is interrupted, interrupt all running child commands
        if (interrupted) {
            for (Command command : runningCommands) {
                command.end(true);
            }
        }
        runningCommands.clear();
    }

    // getRequirements() is inherited from CommandGroupBase (which inherits from CommandBase)
    // The requirements are populated in the constructor by calling addRequirement for each subsystem.

    // Optional: Add a method to add commands dynamically if needed,
    // but constructor-based is simpler for typical FTC auto.
    // public void addCommands(Command... commands) { ... }
}
