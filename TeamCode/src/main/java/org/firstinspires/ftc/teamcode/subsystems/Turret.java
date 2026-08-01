package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;
import org.firstinspires.ftc.teamcode.util.PIDFController;

// Servo-driven turret that auto-aims at the goal AprilTag using the Limelight.
//
// It reads the Limelight's cached horizontal error (tx) each loop and runs a
// PD loop that drives tx -> 0 by commanding the CRServo's rotation speed. The
// Limelight and this turret are separate subsystems whose periodic() methods
// both run every scheduler loop, so vision and aiming happen in parallel.
//
// TRAVEL LIMIT. The turret is on a CRServo with wires running to it, so it must
// never wind up more than a single turn. The servo's feedback wire is absolute
// but only WITHIN one revolution -- it reads the same voltage at +170 as it
// does a full turn later -- so this class accumulates that raw reading across
// its 360->0 rollovers into a continuous angle, and it's that continuous angle
// (not the raw one) that the limit is enforced against. See getTravelDeg().
//
// At the limit the turret doesn't just stop: it UNWRAPS, swinging a full turn
// the other way to the same physical heading, which spends the winding without
// losing the aim. It can't shoot mid-unwrap (onTarget goes false) but it comes
// out the far side still pointed at the goal.
public class Turret implements Subsystem {
    public enum State {
        IDLE,             // hold still (servo stopped)
        AUTO_AIM,         // track the goal tag from the Limelight
        MANUAL,           // operator drives it directly with setManualPower()
        RETURN_TO_ORIGIN  // park back at ORIGIN_DEG using the feedback wire
    }

    private final CRServo servo;
    private final AnalogInput encoder;
    private final Limelight limelight;
    private final PIDFController controller =
            new PIDFController(Constants.Turret.kP, Constants.Turret.kI,
                    Constants.Turret.kD, Constants.Turret.kF);

    private State state = State.IDLE;
    private double manualPower = 0;
    private boolean onTarget = false;

    // Raw feedback as of the last periodic(), used to spot a rollover.
    private double lastRawDeg;
    // Raw reading accumulated across rollovers, so it keeps counting past a full
    // turn: this is what knows the difference between "+170" and "wound a whole
    // turn and now at +170".
    private double continuousDeg;
    // The continuous angle that means "straight ahead", and the ORIGIN_DEG it
    // was built from (so a live edit on Panels re-references it).
    private double originContinuousDeg;
    private double originBaselineDeg;

    // An unwrap in progress, and the travel it's heading for.
    private boolean unwinding = false;
    private double unwindTargetDeg = 0;
    // Set after an unwrap, cleared once travel is comfortably back inside the
    // limit. Stops a goal sitting right on the boundary from unwrapping forever.
    private boolean unwindGuarded = false;

    public Turret(Hardware hardware, Limelight limelight) {
        servo = hardware.turret;
        encoder = hardware.turretEncoder;
        this.limelight = limelight;

        // Seed the accumulator from wherever the turret physically is at init.
        // Absolute feedback means this is correct even if someone turned the
        // turret by hand while the robot was off.
        lastRawDeg = getAngleDeg();
        continuousDeg = lastRawDeg;
        rebaseOrigin();
    }

    // Raw angle off the servo's feedback wire, 0-360 degrees, absolute within a
    // single revolution. This is the SERVO's shaft angle: it equals the turret's
    // angle only if the servo drives the turret 1:1. Through a reduction, scale
    // it here.
    public double getAngleDeg() {
        return encoder.getVoltage() / encoder.getMaxVoltage() * 360.0;
    }

    // How far the turret has wound off its origin, in degrees, NOT wrapped: past
    // a full turn this keeps growing, which is the whole point. Positive is one
    // way round, negative the other. This is the number the travel limit and the
    // park loop both work in.
    public double getTravelDeg() {
        return continuousDeg - originContinuousDeg;
    }

    // Kept as the old name for telemetry and the tuning opmode. Same number as
    // getTravelDeg() -- an "error from origin" that counts past 180 rather than
    // wrapping, because a turret wound 200 degrees needs to unwind 200, not
    // helpfully take the 160-degree "short way" and wind up a full turn.
    public double getOriginErrorDeg() {
        return getTravelDeg();
    }

    public boolean isAtOrigin() {
        return Math.abs(getTravelDeg()) <= TurretTuning.RETURN_TOLERANCE_DEG;
    }

    // True while swinging a full turn to unwrap. The turret is deliberately off
    // the goal for the whole move, so nothing should try to shoot.
    public boolean isUnwinding() {
        return unwinding;
    }

    // Put the origin on the continuous scale, on the turn the turret is
    // currently on. Called at init, and again if ORIGIN_DEG is edited live --
    // note that re-referencing mid-match would forget the accumulated winding,
    // which is fine at tuning time (turret near the origin) and not something to
    // do during a match.
    private void rebaseOrigin() {
        originBaselineDeg = TurretTuning.ORIGIN_DEG;
        originContinuousDeg = continuousDeg - wrapDeg(lastRawDeg - originBaselineDeg);
    }

    // Fold an angle difference into [-180, 180].
    private static double wrapDeg(double degrees) {
        double d = degrees % 360.0;
        if (d > 180.0) {
            d -= 360.0;
        } else if (d < -180.0) {
            d += 360.0;
        }
        return d;
    }

    public void setState(State newState) {
        if (newState != state) {
            // Clear PD history on any state change so we never carry a stale
            // error/derivative into a fresh aim.
            controller.reset();
            onTarget = false;
            // Abandon any unwrap in progress too. Its target was computed from
            // the travel at the moment it started; parking or hand-nudging the
            // turret in the middle invalidates that, and resuming the swing on
            // the way back to AUTO_AIM would fling it somewhere arbitrary. The
            // limit check will simply start a fresh unwrap if one is still due.
            unwinding = false;
        }
        state = newState;
    }

    public State getState() {
        return state;
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    // True only while auto-aiming AND locked onto the goal within tolerance.
    // This is what the shooter logic gates firing on.
    public boolean isOnTarget() {
        return state == State.AUTO_AIM && onTarget;
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }

    @Override
    public void periodic() {
        updateAngle();

        switch (state) {
            case AUTO_AIM:
                aim();
                break;

            case MANUAL:
                onTarget = false;
                // The driver gets refused at the limit rather than unwrapped:
                // a full-turn swing nobody asked for, under their own thumb,
                // would be alarming. Motion back inward is always allowed.
                double manual = Range.clip(manualPower, -1.0, 1.0);
                servo.setPower(pushesPastLimit(manual) ? 0.0 : manual);
                break;

            case RETURN_TO_ORIGIN:
                onTarget = false;
                returnToOrigin();
                break;

            case IDLE:
            default:
                onTarget = false;
                servo.setPower(0);
                break;
        }
    }

    // Accumulate the raw feedback into a continuous angle, spotting the 360->0
    // rollover as a step of more than half a turn. Safe because the turret moves
    // only a few degrees per loop at these power caps and a ~50 Hz loop -- it
    // would take a jump of over 180 degrees between two reads to miscount, which
    // is far beyond what the servo can physically do in 20 ms.
    private void updateAngle() {
        double raw = getAngleDeg();
        continuousDeg += wrapDeg(raw - lastRawDeg);
        lastRawDeg = raw;

        if (TurretTuning.ORIGIN_DEG != originBaselineDeg) {
            rebaseOrigin();
        }

        // Let another unwrap arm itself once we're back well inside the limit.
        if (unwindGuarded
                && Math.abs(getTravelDeg())
                    <= TurretTuning.MAX_TRAVEL_DEG - Constants.Turret.UNWIND_HYSTERESIS_DEG) {
            unwindGuarded = false;
        }
    }

    // Which way the shaft angle moves for a positive power. The park loop's
    // INVERT_RETURN is exactly this fact about the wiring, so reuse it.
    private static double angleRateSign(double power) {
        double sign = Math.signum(power);
        return Constants.Turret.INVERT_RETURN ? -sign : sign;
    }

    // True if this command would wind the turret further past its travel limit.
    // Motion back toward the origin is never blocked -- refusing that would
    // strand the turret at the stop with no way home.
    private boolean pushesPastLimit(double power) {
        double travel = getTravelDeg();
        if (Math.abs(travel) < TurretTuning.MAX_TRAVEL_DEG) {
            return false;
        }
        return angleRateSign(power) == Math.signum(travel);
    }

    // Proportional drive toward a target on the continuous travel scale.
    // Returns true once it's there. Used for both the park and the unwrap --
    // same loop, different destination and speed cap.
    private boolean driveToTravel(double targetTravelDeg, double maxPower) {
        double error = getTravelDeg() - targetTravelDeg;
        if (Math.abs(error) <= TurretTuning.RETURN_TOLERANCE_DEG) {
            // Arrived. Stop rather than dither around the target.
            servo.setPower(0);
            return true;
        }

        // Negative sign: drive the error toward zero, i.e. an angle above the
        // target has to come back down.
        double output = Range.clip(-error * TurretTuning.RETURN_kP, -maxPower, maxPower);
        if (Constants.Turret.INVERT_RETURN) {
            output = -output;
        }
        // Stiction floor — a command too small to break static friction would
        // leave it parked just short of the target forever.
        if (Math.abs(output) < Constants.Turret.MIN_AIM_POWER) {
            output = Math.copySign(Constants.Turret.MIN_AIM_POWER, output);
        }
        servo.setPower(output);
        return false;
    }

    // Park back at the origin, unwinding whatever the turret has accumulated.
    // Unlike aim(), this never goes blind: the feedback reads an absolute angle,
    // so it works with no target in sight and knows when it has arrived instead
    // of guessing from a timer.
    private void returnToOrigin() {
        driveToTravel(0.0, TurretTuning.MAX_RETURN_POWER);
    }

    // Begin the full-turn swing to the same heading from the other side: a
    // turret wound to +185 goes to -175, which is the same direction in the
    // world with a turn of winding spent.
    private void startUnwind() {
        double travel = getTravelDeg();
        unwindTargetDeg = travel - Math.copySign(360.0, travel);
        unwinding = true;
        unwindGuarded = true;
        onTarget = false;
        controller.reset();
    }

    private void aim() {
        // An unwrap owns the turret until it lands. tx is meaningless while
        // we're deliberately swinging away from the goal.
        if (unwinding) {
            onTarget = false;
            if (driveToTravel(unwindTargetDeg, Constants.Turret.MAX_UNWIND_POWER)) {
                unwinding = false;
                controller.reset();
            }
            return;
        }

        if (!limelight.hasTarget()) {
            // No goal in view: stop and reset the loop. Do NOT keep driving —
            // a blind CRServo would sweep until it hits a hard stop or twists
            // the wiring.
            onTarget = false;
            controller.reset();
            servo.setPower(0);
            return;
        }

        double tx = limelight.getTx();
        onTarget = Math.abs(tx) <= Constants.Turret.AIM_TOLERANCE_DEG;
        if (onTarget) {
            // Close enough: stop so we don't buzz back and forth around center.
            controller.reset();
            servo.setPower(0);
            return;
        }

        // Pull the latest gains so edits made live on the Panels dashboard take
        // effect this loop without a re-deploy.
        controller.setCoefficients(TurretTuning.kP, TurretTuning.kI,
                TurretTuning.kD, TurretTuning.kF);

        // Setpoint is tx = 0; measurement is the current tx. The controller
        // returns a power whose sign turns us back toward center.
        double output = controller.calculate(0.0, tx);
        output = Range.clip(output, -Constants.Turret.MAX_AIM_POWER, Constants.Turret.MAX_AIM_POWER);
        if (Constants.Turret.INVERT_OUTPUT) {
            output = -output;
        }
        // Floor small commands past the servo's stiction so it actually moves.
        if (Math.abs(output) < Constants.Turret.MIN_AIM_POWER) {
            output = Math.copySign(Constants.Turret.MIN_AIM_POWER, output);
        }

        // The aim is asking us to wind past a full turn. Take the same heading
        // from the other side instead of following the goal round and round.
        if (pushesPastLimit(output)) {
            if (!unwindGuarded) {
                startUnwind();
                aim();  // start the swing this loop rather than idling one.
                return;
            }
            // Already unwrapped once and still pinned at the limit: the goal is
            // sitting right on the boundary. Hold here rather than spinning back
            // and forth across it. onTarget stays false, so nothing fires.
            onTarget = false;
            servo.setPower(0);
            return;
        }

        servo.setPower(output);
    }

    public void stop() {
        setState(State.IDLE);
        servo.setPower(0);
    }
}
