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

    public Turret(Hardware hardware, Limelight limelight) {
        servo = hardware.turret;
        encoder = hardware.turretEncoder;
        this.limelight = limelight;
    }

    // Raw angle off the servo's feedback wire, 0-360 degrees, absolute. This is
    // the SERVO's shaft angle: it equals the turret's angle only if the servo
    // drives the turret 1:1. Through a reduction, scale it here.
    public double getAngleDeg() {
        return encoder.getVoltage() / encoder.getMaxVoltage() * 360.0;
    }

    // How far we are from the origin, as a shortest-path error in
    // [-180, 180]. Positive means the current angle sits above the origin.
    public double getOriginErrorDeg() {
        return wrapDeg(getAngleDeg() - TurretTuning.ORIGIN_DEG);
    }

    public boolean isAtOrigin() {
        return Math.abs(getOriginErrorDeg()) <= TurretTuning.RETURN_TOLERANCE_DEG;
    }

    // Fold an angle difference into [-180, 180] so the turret always takes the
    // short way round instead of unwinding 350 degrees to move 10.
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
        switch (state) {
            case AUTO_AIM:
                aim();
                break;

            case MANUAL:
                onTarget = false;
                servo.setPower(Range.clip(manualPower, -1.0, 1.0));
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

    // Drive the feedback angle back to ORIGIN_DEG and hold there. Unlike aim(),
    // this never goes blind: the feedback wire reads an absolute angle, so it
    // works with no target in sight, and it knows when it has arrived instead of
    // guessing from a timer.
    private void returnToOrigin() {
        double error = getOriginErrorDeg();
        if (Math.abs(error) <= TurretTuning.RETURN_TOLERANCE_DEG) {
            // Home. Stop rather than dither around the origin.
            servo.setPower(0);
            return;
        }

        // Negative sign: drive the error toward zero, i.e. an angle above the
        // origin has to come back down.
        double output = Range.clip(-error * TurretTuning.RETURN_kP,
                -TurretTuning.MAX_RETURN_POWER, TurretTuning.MAX_RETURN_POWER);
        if (Constants.Turret.INVERT_RETURN) {
            output = -output;
        }
        // Same stiction floor as the aim loop — a command too small to break
        // static friction would leave it parked just short of home forever.
        if (Math.abs(output) < Constants.Turret.MIN_AIM_POWER) {
            output = Math.copySign(Constants.Turret.MIN_AIM_POWER, output);
        }
        servo.setPower(output);
    }

    private void aim() {
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
        servo.setPower(output);
    }

    public void stop() {
        setState(State.IDLE);
        servo.setPower(0);
    }
}
