package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable turret aim PID gains, editable from the Panels dashboard while
 * the robot is running (no re-deploy).
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
    public static double kP = Constants.Turret.kP;
    public static double kI = Constants.Turret.kI;
    public static double kD = Constants.Turret.kD;
    public static double kF = Constants.Turret.kF;

    // Where "straight ahead" reads on the servo's feedback wire, in degrees.
    // Jog this on Panels until parking actually lands the turret dead centre,
    // then copy it back into Constants.Turret.ORIGIN_DEG.
    public static double ORIGIN_DEG = Constants.Turret.ORIGIN_DEG;

    // The return-to-origin park loop, shared with the unwrap swing.
    public static double RETURN_kP = Constants.Turret.RETURN_kP;
    public static double MAX_RETURN_POWER = Constants.Turret.MAX_RETURN_POWER;
    public static double RETURN_TOLERANCE_DEG = Constants.Turret.RETURN_TOLERANCE_DEG;

    // How far either side of the origin the turret may wind before it unwraps.
    // 180 = one full turn of total travel. Lower it if the wiring says so;
    // raising it past 180 lets the turret wind more than a turn.
    public static double MAX_TRAVEL_DEG = Constants.Turret.MAX_TRAVEL_DEG;
    public static double MAX_UNWIND_POWER = Constants.Turret.MAX_UNWIND_POWER;
    public static double UNWIND_HYSTERESIS_DEG = Constants.Turret.UNWIND_HYSTERESIS_DEG;

    // If the turret drives AWAY from the target when auto-aiming, flip this.
    // (Camera-mounting sign: tx versus servo power.)
    public static boolean INVERT_OUTPUT = Constants.Turret.INVERT_OUTPUT;

    // If the turret runs AWAY from the origin when parking, flip this.
    // (Servo-gearing sign: feedback angle versus servo power.) Independent of
    // INVERT_OUTPUT — they are two separate facts about how the robot is built.
    public static boolean INVERT_RETURN = Constants.Turret.INVERT_RETURN;

    // Aim loop shaping. AIM_TOLERANCE_DEG and MIN_AIM_POWER interact: if the
    // smallest power the servo will accept swings the turret further than the
    // tolerance band is wide, it can never settle, and it will hunt back and
    // forth across the target forever. Widen the band or lower the floor.
    public static double AIM_TOLERANCE_DEG = Constants.Turret.AIM_TOLERANCE_DEG;
    public static double MAX_AIM_POWER = Constants.Turret.MAX_AIM_POWER;
    public static double MIN_AIM_POWER = Constants.Turret.MIN_AIM_POWER;

    // Speed of a manual dpad nudge.
    public static double MANUAL_NUDGE_POWER = Constants.Turret.MANUAL_NUDGE_POWER;
}
