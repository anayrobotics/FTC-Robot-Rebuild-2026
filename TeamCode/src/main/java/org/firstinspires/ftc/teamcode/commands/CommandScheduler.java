package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

// Minimal command scheduler. Register subsystems once, schedule commands as
// needed, then call run() every loop. A scheduled command that requires a
// subsystem already in use cancels the command currently using it.
public final class CommandScheduler {
    private static CommandScheduler instance;

    private final Set<Subsystem> subsystems = new LinkedHashSet<>();
    private final List<Command> running = new ArrayList<>();
    private final Map<Subsystem, Command> requiring = new HashMap<>();

    private CommandScheduler() {}

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }

    public void registerSubsystem(Subsystem... toAdd) {
        for (Subsystem s : toAdd) {
            subsystems.add(s);
        }
    }

    public void schedule(Command command) {
        // Free up any subsystem this command needs by cancelling its current owner.
        for (Subsystem requirement : command.getRequirements()) {
            Command owner = requiring.get(requirement);
            if (owner != null) {
                cancel(owner);
            }
        }
        for (Subsystem requirement : command.getRequirements()) {
            requiring.put(requirement, command);
        }
        running.add(command);
        command.initialize();
    }

    public void cancel(Command command) {
        if (!running.remove(command)) {
            return;
        }
        command.end(true);
        requiring.values().removeAll(java.util.Collections.singleton(command));
    }

    public void run() {
        for (Subsystem s : subsystems) {
            s.periodic();
        }

        for (Command command : new ArrayList<>(running)) {
            command.execute();
            if (command.isFinished()) {
                running.remove(command);
                requiring.values().removeAll(java.util.Collections.singleton(command));
                command.end(false);
            }
        }
    }

    public void cancelAll() {
        for (Command command : new ArrayList<>(running)) {
            cancel(command);
        }
    }

    // Clears all state. Call at OpMode init so subsystems from a previous run
    // (the singleton survives across runs) don't linger.
    public void reset() {
        running.clear();
        requiring.clear();
        subsystems.clear();
    }
}
