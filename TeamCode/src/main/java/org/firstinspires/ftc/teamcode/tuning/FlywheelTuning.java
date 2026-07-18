package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable flywheel velocity PIDF gains, editable from the Panels
 * dashboard while the robot is running (no re-deploy).
 *
 * <p>Fields must be {@code public static} and <b>non-final</b> for Panels to
 * expose them. They are seeded from {@link Constants.Flywheel} so that class
 * stays the source of the compile-time defaults; once you dial values in on
 * Panels, copy the good numbers back into {@link Constants.Flywheel}.
 *
 * <p>{@link org.firstinspires.ftc.teamcode.subsystems.Flywheel} reads these
 * every loop, so edits take effect immediately.
 */
@Configurable
public class FlywheelTuning {
    public static double kP = Constants.Flywheel.kP;
    public static double kI = Constants.Flywheel.kI;
    public static double kD = Constants.Flywheel.kD;
    public static double kF = Constants.Flywheel.kF;

    // Setpoint the flywheel tuning OpMode spins to while you tune the gains.
    public static double TEST_RPM = Constants.Flywheel.SHOOT_RPM;
}
