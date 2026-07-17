package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.PIDFController;

// Servo-driven turret that auto-aims at the goal AprilTag using the Limelight.
//
// It reads the Limelight's cached horizontal error (tx) each loop and runs a
// PD loop that drives tx -> 0 by commanding the CRServo's rotation speed. The
// Limelight and this turret are separate subsystems whose periodic() methods
// both run every scheduler loop, so vision and aiming happen in parallel.
public class Turret implements Subsystem {
    public enum State {
        IDLE,      // hold still (servo stopped)
        AUTO_AIM,  // track the goal tag from the Limelight
        MANUAL     // operator drives it directly with setManualPower()
    }

    private final CRServo servo;
    private final Limelight limelight;
    private final PIDFController controller =
            new PIDFController(Constants.Turret.kP, Constants.Turret.kI,
                    Constants.Turret.kD, Constants.Turret.kF);

    private State state = State.IDLE;
    private double manualPower = 0;
    private boolean onTarget = false;

    public Turret(Hardware hardware, Limelight limelight) {
        servo = hardware.turret;
        this.limelight = limelight;
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

            case IDLE:
            default:
                onTarget = false;
                servo.setPower(0);
                break;
        }
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
