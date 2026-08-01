package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tuning.StopperTuning;

// Two-position gate servo sitting between the indexer and the flywheel. It only
// ever does one thing: hold the next ball back (BLOCKING) or get out of the way
// (OPEN). Commands set the state; periodic() writes the matching servo position.
//
// Positions are read from StopperTuning every loop rather than latched at the
// state change, so nudging a value on Panels moves the gate right away.
//
// Like the hood, a positional servo has no feedback — we can't know the gate
// actually got there. isSettled() is just a travel timer since the last state
// change, so callers can wait out the motion before feeding a ball into it.
public class Stopper implements Subsystem {
    public enum State {
        BLOCKING,
        OPEN
    }

    private final Servo servo;
    private State state = State.BLOCKING;

    // Time since the last state CHANGE (not since the last setState call), used
    // by isSettled(). Starts elapsed so we're considered settled at init.
    private final ElapsedTime sinceChange = new ElapsedTime();

    public Stopper(Hardware hardware) {
        servo = hardware.stopper;
        // Hardware.init() already parked the gate closed; match that so our
        // first periodic() doesn't command a move.
        sinceChange.reset();
    }

    public void setState(State state) {
        if (this.state != state) {
            this.state = state;
            sinceChange.reset();
        }
    }

    public State getState() {
        return state;
    }

    // Convenience wrappers — most callers just want "let the ball through" or
    // "hold it back" without naming the enum.
    public void open() {
        setState(State.OPEN);
    }

    public void block() {
        setState(State.BLOCKING);
    }

    public boolean isOpen() {
        return state == State.OPEN;
    }

    // True once enough time has passed for the servo to have finished travelling
    // to the current state. Gate a shot on this so we don't feed into a gate
    // that's still swinging open.
    public boolean isSettled() {
        return sinceChange.seconds() >= StopperTuning.TRAVEL_TIME_S;
    }

    // The position being commanded for the current state.
    public double getCommandedPosition() {
        return state == State.OPEN
                ? StopperTuning.OPEN_POSITION
                : StopperTuning.BLOCKING_POSITION;
    }

    @Override
    public void periodic() {
        servo.setPosition(getCommandedPosition());
    }

    // Fail safe: leave the gate closed so a ball can't drift into the flywheel
    // as it coasts down.
    public void stop() {
        block();
    }
}
