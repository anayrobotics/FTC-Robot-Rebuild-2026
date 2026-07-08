package org.firstinspires.ftc.teamcode.commands;

import java.util.Collections;
import java.util.Set;

// A unit of work run by the CommandScheduler. A command initializes, executes
// each loop until isFinished(), then ends. Requirements declare which subsystems
// it uses so the scheduler can prevent two commands fighting over the same one.
public interface Command {
    default void initialize() {}

    default void execute() {}

    default boolean isFinished() { return false; }

    default void end(boolean interrupted) {}

    default Set<Subsystem> getRequirements() { return Collections.emptySet(); }
}
