package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import java.util.Collections;
import java.util.Set;

// Sets the indexer to a target state and finishes immediately.
public class SetIndexerStateCommand implements Command {
    private final Indexer indexer;
    private final Indexer.State target;

    public SetIndexerStateCommand(Indexer indexer, Indexer.State target) {
        this.indexer = indexer;
        this.target = target;
    }

    @Override
    public void initialize() {
        indexer.setState(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(indexer);
    }
}
