package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable drive-to-pose PID gains, editable from the Panels dashboard while
 * the robot is running (no re-deploy).
 *
 * <p>Fields must be {@code public static} and <b>non-final</b> for Panels to
 * expose them. They are seeded from {@link Constants.DriveToPose} so that class
 * stays the source of the compile-time defaults; once you dial values in on
 * Panels, copy the good numbers back into {@link Constants.DriveToPose}.
 *
 * <p>{@link org.firstinspires.ftc.teamcode.subsystems.DriveToPose} reads the
 * gains every loop, so edits take effect immediately. {@code TARGET_*} is the
 * pose the tuning OpMode drives to while you hold A — nudge it around the field
 * to test forward, strafe, diagonal, and turning moves.
 */
@Configurable
public class DriveTuning {
    // Robot-forward translation PID (inches of error -> drive power).
    public static double FORWARD_kP = Constants.DriveToPose.FORWARD_kP;
    public static double FORWARD_kI = Constants.DriveToPose.FORWARD_kI;
    public static double FORWARD_kD = Constants.DriveToPose.FORWARD_kD;

    // Robot-strafe translation PID (inches of error -> drive power).
    public static double STRAFE_kP = Constants.DriveToPose.STRAFE_kP;
    public static double STRAFE_kI = Constants.DriveToPose.STRAFE_kI;
    public static double STRAFE_kD = Constants.DriveToPose.STRAFE_kD;

    // Heading PID (radians of error -> turn power).
    public static double HEADING_kP = Constants.DriveToPose.HEADING_kP;
    public static double HEADING_kI = Constants.DriveToPose.HEADING_kI;
    public static double HEADING_kD = Constants.DriveToPose.HEADING_kD;

    // Target pose the tuning OpMode drives to, in PedroPathing field coordinates
    // (inches; heading in degrees). Edit live on Panels to test different moves.
    public static double TARGET_X = 24.0;
    public static double TARGET_Y = 0.0;
    public static double TARGET_HEADING_DEG = 0.0;
}
