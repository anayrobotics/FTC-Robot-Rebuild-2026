package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import java.util.Collections;
import java.util.Set;

// Sets the flywheel target RPM and finishes immediately. The subsystem's PID
// loop holds that RPM until another command changes it.
public class SetFlywheelRpmCommand implements Command {
    private final Flywheel flywheel;
    private final double targetRpm;

    public SetFlywheelRpmCommand(Flywheel flywheel, double targetRpm) {
        this.flywheel = flywheel;
        this.targetRpm = targetRpm;
    }

    @Override
    public void initialize() {
        flywheel.setTargetRpm(targetRpm);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(flywheel);
    }
}
