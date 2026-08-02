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
 * position a little each frame until tx reaches zero. If the tag is lost, the
 * turret is commanded back to its configured neutral input, rather than holding
 * an unknown direction at a mechanical limit.
 *
 * <h2>Why the aim is quiet when parked</h2>
 * A naive version of this loop visibly twitches on a stationary robot. Three
 * separate causes, all handled here:
 * <ul>
 *   <li><b>Per-frame, not per-loop.</b> The camera produces
 *       {@link Constants.Vision#FRAME_RATE_FPS} frames a second and the OpMode
 *       loop runs several times faster, so most loops see a measurement they've
 *       already acted on. Because {@code kP} saturates
 *       {@link Constants.Turret#MAX_AIM_RATE} for any error past a fraction of a
 *       degree, re-integrating a stale error slews at full rate until it
 *       overshoots and the next frame slews it back — a limit cycle. The aim
 *       only advances on a fresh frame, timed by the gap between frames.</li>
 *   <li><b>Filtered tx.</b> The AprilTag corner solve jitters a few tenths of a
 *       degree with everything dead still; {@link Constants.Turret#TX_FILTER_ALPHA}
 *       smooths it.</li>
 *   <li><b>Lock hysteresis and a grace window.</b> The lock engages at
 *       AIM_TOLERANCE_DEG and releases at the wider KEEP_AIM_TOLERANCE_DEG, and
 *       a tag missing for fewer than {@link Constants.Turret#TARGET_GRACE_FRAMES}
 *       frames freezes the turret instead of snapping it to neutral and back.</li>
 * </ul>
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
    // Time since the aim was last integrated. This is NOT the loop period: the
    // aim only advances on a fresh camera frame, so the correct dt is the gap
    // between frames, not between loops.
    private final ElapsedTime sinceAimUpdate = new ElapsedTime();
    // Time since the goal tag was last actually seen, for the grace window.
    private final ElapsedTime sinceTargetSeen = new ElapsedTime();

    private State state = State.IDLE;
    // Normalized manual direction, not direct electrical CR-servo power.
    private double manualPower = 0;
    private double commandedPosition = Constants.Turret.NEUTRAL_POSITION;
    private boolean onTarget = false;

    // Low-pass state for tx. filterPrimed distinguishes "filter holds a real
    // measurement" from "filter has never been fed", so the first sample after
    // acquiring a tag seeds the filter outright instead of ramping up from zero.
    private double filteredTx = 0;
    private boolean filterPrimed = false;
    // True while we hold an aim we're still willing to act on — the tag is
    // visible, or it went missing recently enough to be inside the grace window.
    private boolean aimValid = false;

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
            // Drop the vision history. Coming back into AUTO_AIM after a park or
            // a manual nudge, the old filter contents and grace timer describe
            // where the turret used to be pointed and must not be acted on.
            filterPrimed = false;
            aimValid = false;
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

    /**
     * Current absolute horizontal tag error, or MAX_VALUE with no usable aim.
     *
     * <p>Reports the FILTERED error, and keeps reporting it through the grace
     * window rather than jumping to MAX_VALUE the instant one frame drops. That
     * matters beyond the turret itself: the OpModes gate a firing burst on this
     * staying under KEEP_AIM_TOLERANCE_DEG, so a single dropped frame used to
     * cut the feed mid-shot.
     */
    public double getAimErrorDeg() {
        if (state != State.AUTO_AIM || !aimValid) {
            return Double.MAX_VALUE;
        }
        return Math.abs(filteredTx);
    }

    /** Raw per-frame visibility: is the goal tag in the newest frame? */
    public boolean hasTarget() {
        return limelight != null && limelight.hasTarget();
    }

    /**
     * Whether the turret currently holds an aim it's willing to act on — either
     * the tag is visible now, or it dropped out recently enough to be inside the
     * grace window. Differs from {@link #hasTarget()} only during that window.
     */
    public boolean hasUsableAim() {
        return aimValid;
    }

    /** Filtered tx actually driving the aim loop. For telemetry and tuning. */
    public double getFilteredTx() {
        return filteredTx;
    }

    @Override
    public void periodic() {
        // A long first loop after INIT must not turn into a large position jump.
        double dt = Math.min(loopTimer.seconds(), 0.05);
        loopTimer.reset();

        switch (state) {
            case AUTO_AIM:
                aim();
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

    // No dt parameter: the aim advances on camera frames, not loops, so it times
    // its own interval with sinceAimUpdate rather than using the loop period.
    private void aim() {
        boolean freshFrame;

        if (hasTarget()) {
            sinceTargetSeen.reset();
            aimValid = true;
            freshFrame = limelight.isNewFrame();

            // Only fold a measurement into the filter once. Running the filter
            // every loop instead would make its time constant depend on the loop
            // rate, and would pull it toward a value the camera hasn't updated.
            if (freshFrame) {
                double rawTx = limelight.getTx();
                filteredTx = filterPrimed
                        ? filteredTx + TurretTuning.TX_FILTER_ALPHA * (rawTx - filteredTx)
                        : rawTx;
                filterPrimed = true;
            }
        } else if (aimValid && sinceTargetSeen.seconds() <= TurretTuning.TARGET_GRACE_S) {
            // Brief dropout. Freeze: hold the last commanded position, the last
            // filtered error, and the lock flag. Deliberately NOT integrating
            // the stale error — that's how you walk off target while blind.
            sinceAimUpdate.reset();
            writePosition();
            return;
        } else {
            // Genuinely lost — the tag left the frame, or the camera stopped.
            // Neutral is a known, safe pose and keeps the linkage off its
            // internal limits, which is better than holding an unverifiable aim.
            onTarget = false;
            aimValid = false;
            filterPrimed = false;
            sinceAimUpdate.reset();
            commandNeutral();
            return;
        }

        // Hysteresis: it takes AIM_TOLERANCE_DEG to claim the lock but the wider
        // KEEP_AIM_TOLERANCE_DEG to lose it. One shared threshold let tx noise
        // straddling the edge toggle the lock, and every toggle spent a nudge.
        double lockBand = onTarget
                ? TurretTuning.KEEP_AIM_TOLERANCE_DEG
                : TurretTuning.AIM_TOLERANCE_DEG;
        onTarget = Math.abs(filteredTx) <= lockBand;
        if (onTarget) {
            // The Axon's internal position loop holds the last good aim command.
            sinceAimUpdate.reset();
            writePosition();
            return;
        }

        // Nothing new to act on: the camera hasn't produced a frame since the
        // last correction. Acting anyway is the flinch — kP saturates the rate
        // limiter for any error past a fraction of a degree, so re-integrating a
        // stale error just slews at full speed until it overshoots and the next
        // frame slews it back.
        if (!freshFrame) {
            writePosition();
            return;
        }

        // Time between FRAMES, not between loops, since that's the interval this
        // correction is actually covering. Clamped so a dropped frame or a slow
        // first loop can't turn into an oversized jump.
        double frameDt = Math.min(sinceAimUpdate.seconds(), 2.0 / Constants.Vision.FRAME_RATE_FPS);
        sinceAimUpdate.reset();

        // tx is positive when the tag is right in the image. The default sign
        // assumes decreasing Servo position turns the camera right; flip
        // INVERT_OUTPUT if the first on-robot test proves the opposite.
        double rate = -filteredTx * TurretTuning.kP;
        if (TurretTuning.INVERT_OUTPUT) {
            rate = -rate;
        }
        rate = Range.clip(rate, -TurretTuning.MAX_AIM_RATE, TurretTuning.MAX_AIM_RATE);
        commandedPosition = clipPosition(commandedPosition + rate * frameDt);
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
        aimValid = false;
        filterPrimed = false;
        commandNeutral();
    }
}
