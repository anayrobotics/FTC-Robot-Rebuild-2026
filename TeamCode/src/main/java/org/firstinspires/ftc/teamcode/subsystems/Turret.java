package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * Positional-servo turret that centers the Limelight on the selected AprilTag.
 *
 * <p>The Axon is programmed in Servo Mode, so {@link Servo#setPosition(double)}
 * commands a bounded angle that the servo holds. The Axon's programmed left and
 * right limits are the wiring safety boundary; this class never commands the
 * old CR-servo full-turn/unwrap behavior.
 *
 * <p>Auto aim is a position-rate loop. A nonzero Limelight tx nudges the held
 * position a little each loop until tx reaches zero. If the tag is lost, the
 * turret is commanded back to its configured neutral input, rather than holding
 * an unknown direction at a mechanical limit.
 */
public class Turret implements Subsystem {
    public enum State {
        IDLE,
        AUTO_AIM,
        MANUAL,
        RETURN_TO_ORIGIN
    }

    private final Servo servo;
    private final Limelight limelight;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private State state = State.IDLE;
    // Normalized manual direction, not direct electrical CR-servo power.
    private double manualPower = 0;
    private double commandedPosition = Constants.Turret.NEUTRAL_POSITION;
    private boolean onTarget = false;

    public Turret(Hardware hardware) {
        this(hardware, null);
    }

    public Turret(Hardware hardware, Limelight limelight) {
        servo = hardware.turret;
        this.limelight = limelight;
        loopTimer.reset();
    }

    public void setState(State newState) {
        if (newState != state) {
            onTarget = false;
        }
        state = newState;
    }

    public State getState() {
        return state;
    }

    /**
     * Sets normalized manual direction in [-1, 1]. The command is integrated at
     * MANUAL_NUDGE_RATE; it is not sent directly as CR-servo power.
     */
    public void setManualPower(double power) {
        manualPower = Range.clip(power, -1.0, 1.0);
    }

    /** The last standard FTC Servo input position commanded to the Axon. */
    public double getCommandedPosition() {
        return commandedPosition;
    }

    /** True only while auto aiming and the selected tag is within the lock band. */
    public boolean isOnTarget() {
        return state == State.AUTO_AIM && onTarget;
    }

    /** Current absolute horizontal tag error, or MAX_VALUE without a target. */
    public double getAimErrorDeg() {
        if (state != State.AUTO_AIM || !hasTarget()) {
            return Double.MAX_VALUE;
        }
        return Math.abs(limelight.getTx());
    }

    public boolean hasTarget() {
        return limelight != null && limelight.hasTarget();
    }

    @Override
    public void periodic() {
        // A long first loop after INIT must not turn into a large position jump.
        double dt = Math.min(loopTimer.seconds(), 0.05);
        loopTimer.reset();

        switch (state) {
            case AUTO_AIM:
                aim(dt);
                break;

            case MANUAL:
                onTarget = false;
                commandedPosition = clipPosition(commandedPosition
                        + manualPower * TurretTuning.MANUAL_NUDGE_RATE * dt);
                writePosition();
                break;

            case RETURN_TO_ORIGIN:
                onTarget = false;
                commandNeutral();
                break;

            case IDLE:
            default:
                // Leave the last position alone during INIT. A positional servo
                // will hold its prior command; we deliberately do not create a
                // new one until an active state asks for it.
                onTarget = false;
                break;
        }
    }

    private void aim(double dt) {
        if (!hasTarget()) {
            // A tag outside the turret's limited swing eventually leaves the
            // image. Returning neutral gives the driver a known, safe pose and
            // avoids keeping the linkage pressed against an internal limit.
            onTarget = false;
            commandNeutral();
            return;
        }

        double tx = limelight.getTx();
        onTarget = Math.abs(tx) <= TurretTuning.AIM_TOLERANCE_DEG;
        if (onTarget) {
            // The Axon's internal position loop holds the last good aim command.
            writePosition();
            return;
        }

        // tx is positive when the tag is right in the image. The default sign
        // assumes decreasing Servo position turns the camera right; flip
        // INVERT_OUTPUT if the first on-robot test proves the opposite.
        double rate = -tx * TurretTuning.kP;
        if (TurretTuning.INVERT_OUTPUT) {
            rate = -rate;
        }
        rate = Range.clip(rate, -TurretTuning.MAX_AIM_RATE, TurretTuning.MAX_AIM_RATE);
        commandedPosition = clipPosition(commandedPosition + rate * dt);
        writePosition();
    }

    private void commandNeutral() {
        commandedPosition = clipPosition(TurretTuning.NEUTRAL_POSITION);
        writePosition();
    }

    private void writePosition() {
        servo.setPosition(commandedPosition);
    }

    private static double clipPosition(double position) {
        return Range.clip(position, TurretTuning.MIN_POSITION, TurretTuning.MAX_POSITION);
    }

    /** On OpMode stop, return to the known neutral input instead of holding aim. */
    public void stop() {
        state = State.IDLE;
        onTarget = false;
        commandNeutral();
    }
}
