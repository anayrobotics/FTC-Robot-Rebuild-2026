package org.firstinspires.ftc.teamcode.commands;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

// Runs an action once and finishes immediately. Handy for one-shot state changes.
public class InstantCommand implements Command {
    private final Runnable action;
    private final Set<Subsystem> requirements;

    public InstantCommand(Runnable action, Subsystem... requirements) {
        this.action = action;
        this.requirements = new HashSet<>(Arrays.asList(requirements));
    }

    @Override
    public void initialize() {
        action.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }
}
