package org.firstinspires.ftc.teamcode.commands.base;

public class InstantCommand extends CommandBase {
    private final Runnable toRun;

    public InstantCommand(Runnable toRun) {
        this.toRun = toRun;
    }

    @Override
    public void initialize() {
        if (toRun != null) {
            toRun.run();
        }
    }

    @Override
    public void execute() {
        // No ongoing execution for an instant command
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
