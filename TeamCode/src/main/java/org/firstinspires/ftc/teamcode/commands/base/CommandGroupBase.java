package org.firstinspires.ftc.teamcode.commands.base;

import org.firstinspires.ftc.teamcode.subsystems.base.SubsystemBase;
import java.util.HashSet;
import java.util.Set;

// Basic abstract class for command groups
public abstract class CommandGroupBase extends CommandBase {
    protected final Set<SubsystemBase> requirements = new HashSet<>();

    @Override
    public Set<SubsystemBase> getRequirements() {
        return requirements;
    }
}
