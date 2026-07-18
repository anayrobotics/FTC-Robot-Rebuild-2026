package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tuning.HoodTuning;

// Positional-servo shooter hood that sets the ball's launch angle. Commands set
// a target servo position; periodic() clamps it to the safe travel band and
// writes it to the servo, which holds that angle until it's changed.
//
// Like the flywheel, the hood can auto-range: given a distance to the goal it
// interpolates the tuned distance->position table so the launch angle tracks
// the shot. It has no position feedback (a servo just goes where it's told), so
// there is no "atTarget" check — firing stays gated on the turret and flywheel.
public class Hood implements Subsystem {
    private final Servo servo;

    // Last requested position, BEFORE clamping. periodic() clamps against the
    // live travel band so tightening the bounds on Panels re-clamps a held
    // target immediately.
    private double targetPosition;

    public Hood(Hardware hardware) {
        servo = hardware.hood;
        targetPosition = Constants.Hood.DEFAULT_POSITION;
    }

    // Command a raw servo position (0..1). The actual value written is clamped
    // to [MIN_POSITION, MAX_POSITION] in periodic().
    public void setPosition(double position) {
        targetPosition = position;
    }

    // Snap to the operator presets (flatter close shot / steeper far shot).
    public void setNearPreset() {
        setPosition(HoodTuning.NEAR_PRESET);
    }

    public void setFarPreset() {
        setPosition(HoodTuning.FAR_PRESET);
    }

    // Auto-range: set the hood for a measured goal distance (meters).
    public void setForDistance(double meters) {
        setPosition(positionForDistance(meters));
    }

    // The last position we asked for (pre-clamp).
    public double getTargetPosition() {
        return targetPosition;
    }

    // The position actually being held after clamping to the travel band.
    public double getCommandedPosition() {
        return Range.clip(targetPosition, HoodTuning.MIN_POSITION, HoodTuning.MAX_POSITION);
    }

    // Auto-ranging: given a distance to the goal (meters, e.g. from the
    // Limelight), return the hood position for that shot, linearly interpolated
    // from the tuned table in Constants. Distances outside the table clamp to
    // the nearest end so we always return a sane, in-range position. Mirrors
    // Flywheel.rpmForDistance.
    public static double positionForDistance(double meters) {
        double[] d = Constants.Hood.RANGE_DISTANCES_M;
        double[] p = Constants.Hood.RANGE_POSITIONS;

        if (meters <= d[0]) {
            return p[0];
        }
        if (meters >= d[d.length - 1]) {
            return p[p.length - 1];
        }
        for (int i = 0; i < d.length - 1; i++) {
            if (meters <= d[i + 1]) {
                double t = (meters - d[i]) / (d[i + 1] - d[i]);
                return p[i] + t * (p[i + 1] - p[i]);
            }
        }
        return p[p.length - 1];
    }

    @Override
    public void periodic() {
        // Clamp against the LIVE bounds (from HoodTuning) so a bad preset or a
        // stale auto-range value can never drive the linkage past its stops.
        servo.setPosition(Range.clip(targetPosition,
                HoodTuning.MIN_POSITION, HoodTuning.MAX_POSITION));
    }

    // Return the hood to its stowed default angle.
    public void stop() {
        setPosition(Constants.Hood.DEFAULT_POSITION);
    }
}
