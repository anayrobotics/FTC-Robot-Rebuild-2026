package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live settings for the positional-servo turret. Values are standard FTC Servo
 * input positions/rates, not the numeric settings displayed by the Axon
 * Programmer.
 */
@Configurable
public class TurretTuning {
    /** FTC PWM input that maps to the Axon's programmed neutral. Usually 0.50. */
    public static double NEUTRAL_POSITION = Constants.Turret.NEUTRAL_POSITION;
    public static double MIN_POSITION = Constants.Turret.MIN_POSITION;
    public static double MAX_POSITION = Constants.Turret.MAX_POSITION;

    /** Position units per second per degree of Limelight tx. */
    public static double kP = Constants.Turret.kP;
    public static double MAX_AIM_RATE = Constants.Turret.MAX_AIM_RATE;

    /**
     * Lock hysteresis. The turret declares itself on target inside
     * AIM_TOLERANCE_DEG but doesn't break the lock until the error grows past
     * KEEP_AIM_TOLERANCE_DEG. With a single threshold for both, tx noise sitting
     * right at the band edge toggles the lock on and off, and each drop-out
     * fires another aim nudge — a stationary turret hunting around centre.
     */
    public static double AIM_TOLERANCE_DEG = Constants.Turret.AIM_TOLERANCE_DEG;
    public static double KEEP_AIM_TOLERANCE_DEG = Constants.Turret.KEEP_AIM_TOLERANCE_DEG;

    /** Low-pass on tx, in [0, 1]. 1.0 disables filtering. */
    public static double TX_FILTER_ALPHA = Constants.Turret.TX_FILTER_ALPHA;

    /**
     * How long the tag may be missing before the turret returns to neutral.
     * Derived from a frame count so it scales with the camera: the default is
     * {@link Constants.Turret#TARGET_GRACE_FRAMES} frames at
     * {@link Constants.Vision#FRAME_RATE_FPS}, i.e. 3 / 40 = 75 ms.
     */
    public static double TARGET_GRACE_S =
            Constants.Turret.TARGET_GRACE_FRAMES / Constants.Vision.FRAME_RATE_FPS;

    /** Flip only if the turret moves away from the tag in the image. */
    public static boolean INVERT_OUTPUT = true;

    /** Position units per second for a full dpad manual command. */
    public static double MANUAL_NUDGE_RATE = Constants.Turret.MANUAL_NUDGE_RATE;
}
