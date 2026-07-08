package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Collections;
import java.util.Set;

// Sets the intake to a target state and finishes immediately.
public class SetIntakeStateCommand implements Command {
    private final Intake intake;
    private final Intake.State target;

    public SetIntakeStateCommand(Intake intake, Intake.State target) {
        this.intake = intake;
        this.target = target;
    }

    @Override
    public void initialize() {
        intake.setState(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(intake);
    }
}
