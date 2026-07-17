package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.PIDFController;

// Two-motor flywheel with closed-loop velocity control for exact RPM.
// Commands set a target RPM; periodic() runs the PIDF loop and drives both motors.
public class Flywheel implements Subsystem {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final PIDFController controller =
            new PIDFController(Constants.Flywheel.kP, Constants.Flywheel.kI,
                    Constants.Flywheel.kD, Constants.Flywheel.kF);

    private double targetRpm = 0;
    private double currentRpm = 0;

    public Flywheel(Hardware hardware) {
        left = hardware.flywheelLeft;
        right = hardware.flywheelRight;
    }

    public void setTargetRpm(double rpm) {
        double clamped = Range.clip(rpm, 0, Constants.Flywheel.MAX_RPM);
        // Only reset the loop on a real setpoint jump (e.g. a preset press). Auto-
        // ranging nudges the target a few RPM every loop as the distance estimate
        // jitters; resetting on those tiny changes would wipe the D term each loop.
        if (Math.abs(clamped - targetRpm) > Constants.Flywheel.RPM_TOLERANCE) {
            controller.reset();
        }
        targetRpm = clamped;
    }

    public void stop() {
        setTargetRpm(0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    // Flywheel speed in RPM as of the last periodic() (read from the left encoder).
    public double getCurrentRpm() {
        return currentRpm;
    }

    public boolean atTargetRpm() {
        return targetRpm > 0
                && Math.abs(targetRpm - currentRpm) <= Constants.Flywheel.RPM_TOLERANCE;
    }

    // Auto-ranging: given a distance to the goal (meters, e.g. from the
    // Limelight), return the flywheel RPM to shoot it, linearly interpolated
    // from the tuned table in Constants. Distances outside the table clamp to
    // the nearest end so we always return a sane, in-range RPM.
    public static double rpmForDistance(double meters) {
        double[] d = Constants.Flywheel.RANGE_DISTANCES_M;
        double[] r = Constants.Flywheel.RANGE_RPMS;

        if (meters <= d[0]) {
            return r[0];
        }
        if (meters >= d[d.length - 1]) {
            return r[r.length - 1];
        }
        for (int i = 0; i < d.length - 1; i++) {
            if (meters <= d[i + 1]) {
                double t = (meters - d[i]) / (d[i + 1] - d[i]);
                return r[i] + t * (r[i + 1] - r[i]);
            }
        }
        return r[r.length - 1];
    }

    @Override
    public void periodic() {
        // Sample the encoder once per loop so the PID, atTargetRpm(), and telemetry
        // all use the same reading (and we only hit the hub once).
        currentRpm = left.getVelocity() / Constants.Flywheel.TICKS_PER_REV * 60.0;

        if (targetRpm <= 0) {
            // Let it coast to a stop rather than fighting the PID down to zero.
            left.setPower(0);
            right.setPower(0);
            return;
        }

        double power = Range.clip(controller.calculate(targetRpm, currentRpm), 0, 1);
        left.setPower(power);
        right.setPower(power);
    }
}
