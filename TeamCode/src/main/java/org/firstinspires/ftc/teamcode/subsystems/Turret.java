package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;
import org.firstinspires.ftc.teamcode.util.PIDFController;

// Servo-driven turret that auto-aims at the goal AprilTag using the Limelight.
//
// It reads the Limelight's cached horizontal error (tx) each loop and runs a PD
// loop that drives tx -> 0. The Limelight and this turret are separate
// subsystems whose periodic() methods both run every scheduler loop, so vision
// and aiming happen in parallel.
//
// POSITION, NOT SPEED. The turret is on a positional servo: it is told an angle
// and holds it. So this class keeps one number — commandedDeg, the turret's
// angle either side of straight-ahead — and every state does nothing but move
// that number and write it out. There is no feedback wire and nothing to read
// back, which is fine, because a positional servo can only be where it was told.
//
// The catch is that "where it was told" is only the truth if the servo can keep
// up. That is what MAX_SLEW_DEG_PER_S is for: hold the commanded angle to a rate
// the servo can actually achieve and the command stays a fair proxy for reality.
// Set it too high and commandedDeg becomes a work of fiction — the turret reads
// as parked while it is still swinging, and the travel limits stop meaning
// anything. It is the number to get right.
//
// TRAVEL LIMIT. Just a clamp, applied to commandedDeg in every state. A
// positional servo cannot spin freely, so unlike a CRServo there is no winding
// to accumulate and nothing to unwrap: the turret physically cannot go anywhere
// we don't send it.
//
// STARTING POSITION. Nothing surveys the turret at init, so this class assumes
// it begins pointed straight ahead. Start every match that way — if it is off
// centre, the first command snaps it across.
public class Turret implements Subsystem {
    public enum State {
        IDLE,             // hold the last commanded angle
        AUTO_AIM,         // track the goal tag from the Limelight
        MANUAL,           // operator drives it directly with setManualRate()
        RETURN_TO_ORIGIN  // park back at straight-ahead
    }

    private final Servo servo;
    private final Limelight limelight;
    private final PIDFController controller =
            new PIDFController(Constants.Turret.kP, Constants.Turret.kI,
                    Constants.Turret.kD, Constants.Turret.kF);

    private State state = State.IDLE;
    private double manualRateDegPerSec = 0;
    private boolean onTarget = false;

    // Where we have told the turret to point, in degrees off the origin.
    // Positive is whichever way an increasing servo position swings it (see
    // Constants.Turret.DIRECTION). Assumed 0 at init — see the class comment.
    private double commandedDeg = 0;

    // The ORIGIN_POSITION that commandedDeg is currently measured against, so a
    // live edit on Panels can be spotted. See rebaseOrigin().
    private double originBaseline = TurretTuning.ORIGIN_POSITION;

    // Wall-clock between periodic() calls, so the slew rates are real degrees
    // per second rather than degrees per loop.
    private final ElapsedTime sinceLastLoop = new ElapsedTime();
    private boolean firstLoop = true;

    /**
     * Turret with no camera: manual jogging, parking and the travel limits all
     * work, but AUTO_AIM has nothing to aim at and holds still. Used by the
     * bench tests so the turret can be checked out before the Limelight is
     * wired, and so a camera fault can be ruled out of a turret problem.
     */
    public Turret(Hardware hardware) {
        this(hardware, null);
    }

    public Turret(Hardware hardware, Limelight limelight) {
        servo = hardware.turret;
        this.limelight = limelight;
    }

    // Where the turret is pointing, in degrees off straight-ahead. This is the
    // commanded angle; with MAX_SLEW_DEG_PER_S set honestly it is also, within a
    // servo's settling time, where the turret physically is.
    public double getAngleDeg() {
        return commandedDeg;
    }

    // The servo position that angle corresponds to. Only really of interest to
    // the bench test, which needs it to tell you what to write into
    // ORIGIN_POSITION.
    public double getPosition() {
        return angleToPosition(commandedDeg);
    }

    public boolean isAtOrigin() {
        return Math.abs(commandedDeg) <= TurretTuning.RETURN_TOLERANCE_DEG;
    }

    /**
     * True when the turret is pinned against a software travel limit and the aim
     * wants to go further. The goal is off to the side of everywhere the turret
     * can reach: nothing is broken, but the robot has to turn before this shot
     * is available.
     */
    public boolean isAtLimit() {
        return commandedDeg <= TurretTuning.MIN_ANGLE_DEG + 0.01
                || commandedDeg >= TurretTuning.MAX_ANGLE_DEG - 0.01;
    }

    public void setState(State newState) {
        if (newState != state) {
            // Clear PD history on any state change so we never carry a stale
            // error/derivative into a fresh aim.
            controller.reset();
            onTarget = false;
        }
        state = newState;
    }

    public State getState() {
        return state;
    }

    /** How fast to jog the turret by hand, in degrees per second. */
    public void setManualRate(double degreesPerSecond) {
        manualRateDegPerSec = degreesPerSecond;
    }

    // True only while auto-aiming AND locked onto the goal within tolerance.
    // This is what the shooter logic gates STARTING a shot on.
    public boolean isOnTarget() {
        return state == State.AUTO_AIM && onTarget;
    }

    /**
     * How far off the goal we are right now, in degrees, or a huge number when
     * there is nothing to measure against.
     *
     * <p>Separate from {@link #isOnTarget()} because the two answer different
     * questions. {@code isOnTarget} is a latched band, and the turret stops
     * commanding the moment it's true — so the aim drifts back out, the turret
     * nudges, and the flag flickers true/false around centre even with a steady
     * robot. Gating a burst on that flicker slams the feed shut between every
     * ball. Mid-burst, ask "how far off are we" against a wider band instead.
     */
    public double getAimErrorDeg() {
        if (state != State.AUTO_AIM || !hasTarget()) {
            return Double.MAX_VALUE;
        }
        return Math.abs(limelight.getTx());
    }

    public boolean hasTarget() {
        return limelight != null && limelight.hasTarget();
    }

    /**
     * Re-measure the current angle against a moved origin, so that shifting
     * ORIGIN_POSITION re-labels where the turret is instead of driving it there.
     *
     * <p>Without this, capturing the origin at the turret's current position —
     * which is exactly how test 4a asks you to find it — would move the origin
     * out from under an unchanged commandedDeg and the turret would walk by the
     * same angle again on the very next loop, every time you pressed the button.
     */
    private void rebaseOrigin() {
        double moved = TurretTuning.ORIGIN_POSITION - originBaseline;
        originBaseline = TurretTuning.ORIGIN_POSITION;
        commandedDeg -= moved * servoRangeDeg();
    }

    @Override
    public void periodic() {
        double dt = elapsed();

        if (TurretTuning.ORIGIN_POSITION != originBaseline) {
            rebaseOrigin();
        }

        switch (state) {
            case AUTO_AIM:
                aim(dt);
                break;

            case MANUAL:
                onTarget = false;
                moveAtRate(manualRateDegPerSec, TurretTuning.MANUAL_NUDGE_DEG_PER_S, dt);
                break;

            case RETURN_TO_ORIGIN:
                onTarget = false;
                driveToAngle(0.0, TurretTuning.MAX_RETURN_DEG_PER_S, dt);
                break;

            case IDLE:
            default:
                onTarget = false;
                // Hold wherever we were last told, rather than going limp: a
                // turret that dropped its position on IDLE would sag under the
                // shooter's weight and lose the one thing we know about it.
                hold();
                break;
        }
    }

    /**
     * Seconds since the last periodic(). Clamped because the first loop after
     * INIT, or after the driver station stalls, can hand back a gap of seconds —
     * and a slew rate multiplied by that would step the turret across its whole
     * range in one frame, which is exactly the slam the rate cap exists to stop.
     */
    private double elapsed() {
        double dt = firstLoop ? 0 : sinceLastLoop.seconds();
        sinceLastLoop.reset();
        firstLoop = false;
        return Math.min(dt, 0.1);
    }

    // ------------------------------------------------------------------
    // Commanding the servo. Everything funnels through setAngle(), so the travel
    // limits are enforced in exactly one place and no state can route around
    // them.
    // ------------------------------------------------------------------

    /**
     * SERVO_RANGE_DEG, floored. It is the divisor for every angle-to-position
     * conversion and it is live-editable, so a 0 typed into Panels mid-tune
     * would make the position NaN — and a NaN handed to setPosition() takes the
     * OpMode down mid-match.
     */
    private static double servoRangeDeg() {
        return Math.max(1.0, TurretTuning.SERVO_RANGE_DEG);
    }

    private static double angleToPosition(double angleDeg) {
        double position = TurretTuning.ORIGIN_POSITION + angleDeg / servoRangeDeg();
        // Belt and braces on top of the angle clamp: the servo range or the
        // origin can be mistyped on Panels mid-tune, and a position outside
        // [0, 1] is rejected by the SDK with an exception that would take the
        // whole OpMode down.
        return Range.clip(position, 0.0, 1.0);
    }

    /** Clamp to the travel limits, remember, and write it out. */
    private void setAngle(double angleDeg) {
        commandedDeg = Range.clip(angleDeg,
                TurretTuning.MIN_ANGLE_DEG, TurretTuning.MAX_ANGLE_DEG);
        servo.setPosition(angleToPosition(commandedDeg));
    }

    /**
     * Re-assert the current angle.
     *
     * <p>Every "do nothing" path comes through here rather than simply not
     * writing, so the turret always has holding torque once the OpMode is
     * running. A turret left limp — no target all match, or just sat in IDLE —
     * would flop around under the shooter's weight as the robot drove, and the
     * commanded angle this class is built on would quietly stop being true.
     *
     * <p>Note this is also what ENERGIZES the servo: the hub powers servos up
     * with PWM disabled and the first setPosition() silently re-enables it. That
     * is why nothing is commanded from {@code Hardware.initTurret()} — the
     * turret stays back-driveable until an OpMode actually calls periodic().
     */
    private void hold() {
        setAngle(commandedDeg);
    }

    /** Step the commanded angle at the given rate, capped, for dt seconds. */
    private void moveAtRate(double rateDegPerSec, double maxRateDegPerSec, double dt) {
        double rate = Range.clip(rateDegPerSec, -maxRateDegPerSec, maxRateDegPerSec);
        setAngle(commandedDeg + rate * dt);
    }

    /**
     * Ramp toward a target angle at no more than the given rate. Ramped rather
     * than commanded outright so the servo is never asked for a step it can't
     * physically make in one loop — which is what keeps commandedDeg honest.
     * Returns true once it's there.
     */
    private boolean driveToAngle(double targetDeg, double maxRateDegPerSec, double dt) {
        double error = targetDeg - commandedDeg;
        double step = maxRateDegPerSec * dt;
        if (Math.abs(error) <= step) {
            setAngle(targetDeg);
            return true;
        }
        setAngle(commandedDeg + Math.copySign(step, error));
        return false;
    }

    // ------------------------------------------------------------------
    // The aim loop.
    // ------------------------------------------------------------------

    private void aim(double dt) {
        if (!hasTarget()) {
            // No goal in view (or no camera at all): hold where we are and reset
            // the loop. Do NOT keep sweeping — a blind search would leave the
            // turret pointing somewhere arbitrary the moment the tag came back.
            onTarget = false;
            controller.reset();
            hold();
            return;
        }

        double tx = limelight.getTx();
        onTarget = Math.abs(tx) <= TurretTuning.AIM_TOLERANCE_DEG;
        if (onTarget) {
            // Close enough: stop commanding so we don't hunt back and forth
            // across centre chasing camera noise.
            controller.reset();
            hold();
            return;
        }

        // Pull the latest gains so edits made live on the Panels dashboard take
        // effect this loop without a re-deploy.
        controller.setCoefficients(TurretTuning.kP, TurretTuning.kI,
                TurretTuning.kD, TurretTuning.kF);

        // Setpoint is tx = 0; measurement is the current tx. The controller
        // returns a slew rate in degrees per second whose sign turns us back
        // toward centre.
        double rate = controller.calculate(0.0, tx);
        if (TurretTuning.INVERT_OUTPUT) {
            rate = -rate;
        }
        moveAtRate(rate, TurretTuning.MAX_SLEW_DEG_PER_S, dt);
    }

    public void stop() {
        setState(State.IDLE);
    }
}
