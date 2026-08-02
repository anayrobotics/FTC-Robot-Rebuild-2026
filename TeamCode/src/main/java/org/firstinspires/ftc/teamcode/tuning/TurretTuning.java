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
    public static double AIM_TOLERANCE_DEG = Constants.Turret.AIM_TOLERANCE_DEG;

    /** Flip only if the turret moves away from the tag in the image. */
    public static boolean INVERT_OUTPUT = true;

    /** Position units per second for a full dpad manual command. */
    public static double MANUAL_NUDGE_RATE = Constants.Turret.MANUAL_NUDGE_RATE;
}
