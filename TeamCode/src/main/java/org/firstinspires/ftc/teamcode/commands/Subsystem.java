package org.firstinspires.ftc.teamcode.commands;

// A hardware subsystem. periodic() runs every scheduler loop and is where the
// subsystem should push its current state to the motors.
public interface Subsystem {
    default void periodic() {}
}
