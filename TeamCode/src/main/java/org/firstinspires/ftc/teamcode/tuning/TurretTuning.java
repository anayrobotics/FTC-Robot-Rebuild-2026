package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable turret geometry and aim gains, editable from the Panels dashboard
 * while the robot is running (no re-deploy).
 *
 * <p>Fields must be {@code public static} and <b>non-final</b> for Panels to
 * expose them. Seeded from {@link Constants.Turret}; copy dialed-in values
 * back into that class when you're happy with them.
 *
 * <p>{@link org.firstinspires.ftc.teamcode.subsystems.Turret} reads these every
 * loop, so edits take effect immediately.
 */
@Configurable
public class TurretTuning {
    // Aim gains. Output is a slew rate in DEGREES PER SECOND, not servo power —
    // see Constants.Turret for why the loop is in rate rather than position.
    public static double kP = Constants.Turret.kP;
    public static double kI = Constants.Turret.kI;
    public static double kD = Constants.Turret.kD;
    public static double kF = Constants.Turret.kF;

    // Geometry. Degrees of turret rotation across the servo's full travel, and
    // the position at which it points straight ahead. Jog these on Panels until
    // a commanded angle matches what a protractor says, then copy them back into
    // Constants.Turret.
    public static double SERVO_RANGE_DEG = Constants.Turret.SERVO_RANGE_DEG;
    public static double ORIGIN_POSITION = Constants.Turret.ORIGIN_POSITION;

    // Software travel limits, in degrees off the origin. Every command is
    // clamped to these. Tighten them to the real mechanical range.
    public static double MIN_ANGLE_DEG = Constants.Turret.MIN_ANGLE_DEG;
    public static double MAX_ANGLE_DEG = Constants.Turret.MAX_ANGLE_DEG;

    // If the turret swings AWAY from the target when auto-aiming, flip this.
    // (Camera-mounting sign only — the servo's own geometry sign lives in
    // Constants.Turret.DIRECTION, which is not live-editable.)
    public static boolean INVERT_OUTPUT = Constants.Turret.INVERT_OUTPUT;

    // Rate caps, degrees per second. MAX_SLEW_DEG_PER_S must stay at or below
    // what the servo can actually manage under load: the whole design assumes
    // the commanded angle is a fair proxy for the real one, and that assumption
    // is only as good as this number.
    public static double MAX_SLEW_DEG_PER_S = Constants.Turret.MAX_SLEW_DEG_PER_S;
    public static double MAX_RETURN_DEG_PER_S = Constants.Turret.MAX_RETURN_DEG_PER_S;
    public static double MANUAL_NUDGE_DEG_PER_S = Constants.Turret.MANUAL_NUDGE_DEG_PER_S;

    // Inside this many degrees of the origin, we count as parked.
    public static double RETURN_TOLERANCE_DEG = Constants.Turret.RETURN_TOLERANCE_DEG;

    // Inside this many degrees of the goal we're aimed and stop commanding.
    // Widen it if the turret hunts back and forth across centre instead of
    // settling: that means the smallest useful correction is bigger than the
    // band, so it can never land inside.
    public static double AIM_TOLERANCE_DEG = Constants.Turret.AIM_TOLERANCE_DEG;
}
