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
}
