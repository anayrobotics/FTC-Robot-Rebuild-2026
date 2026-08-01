package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Stopper;

import java.util.Collections;
import java.util.Set;

// Sets the stopper gate to a target state and finishes immediately.
public class SetStopperStateCommand implements Command {
    private final Stopper stopper;
    private final Stopper.State target;

    public SetStopperStateCommand(Stopper stopper, Stopper.State target) {
        this.stopper = stopper;
        this.target = target;
    }

    @Override
    public void initialize() {
        stopper.setState(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(stopper);
    }
}
