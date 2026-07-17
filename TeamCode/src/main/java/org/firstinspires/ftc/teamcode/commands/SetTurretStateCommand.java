package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.Collections;
import java.util.Set;

// Sets the turret to a target state (e.g. AUTO_AIM) and finishes immediately.
// The subsystem's periodic() keeps aiming in that state until something changes
// it. Handy for autonomous, where you'd schedule AUTO_AIM before a shot.
public class SetTurretStateCommand implements Command {
    private final Turret turret;
    private final Turret.State target;

    public SetTurretStateCommand(Turret turret, Turret.State target) {
        this.turret = turret;
        this.target = target;
    }

    @Override
    public void initialize() {
        turret.setState(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.singleton(turret);
    }
}
