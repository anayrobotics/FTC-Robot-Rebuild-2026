package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable hood servo positions, editable from the Panels dashboard while
 * the robot is running (no re-deploy).
 *
 * <p>Fields must be {@code public static} and <b>non-final</b> for Panels to
 * expose them. They are seeded from {@link Constants.Hood} so that class stays
 * the source of the compile-time defaults; once you dial values in on Panels,
 * copy the good numbers back into {@link Constants.Hood}.
 *
 * <p>{@link org.firstinspires.ftc.teamcode.subsystems.Hood} reads the bounds
 * every loop (so tightening them re-clamps immediately), and the hood tuning
 * OpMode commands {@code TEST_POSITION} / the presets directly.
 */
@Configurable
public class HoodTuning {
    // Safe travel band the hood is clamped to every loop.
    public static double MIN_POSITION = Constants.Hood.MIN_POSITION;
    public static double MAX_POSITION = Constants.Hood.MAX_POSITION;

    // Operator presets (flatter close shot / steeper far shot).
    public static double NEAR_PRESET = Constants.Hood.NEAR_PRESET;
    public static double FAR_PRESET = Constants.Hood.FAR_PRESET;

    // Position the hood tuning OpMode drives to while you dial in angles.
    public static double TEST_POSITION = Constants.Hood.DEFAULT_POSITION;
}
