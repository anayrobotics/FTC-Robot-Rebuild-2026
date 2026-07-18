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
}
