package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable camera geometry, editable from the Panels dashboard while the
 * robot is running (no re-deploy).
 *
 * <p>These three numbers are the entire basis of the distance estimate, and
 * therefore of both auto-ranged RPM and auto-ranged hood angle. Get them wrong
 * and every ranged shot is wrong in the same direction — which looks exactly
 * like a badly tuned shooting table, and will send you tuning the wrong thing.
 * Calibrate them FIRST, against a tape measure, using the Limelight test.
 *
 * <p>Fields must be {@code public static} and <b>non-final</b> for Panels to
 * expose them. Seeded from {@link Constants.Vision}; copy dialed-in values back
 * into that class when you're happy with them.
 */
@Configurable
public class VisionTuning {
    /** Lens height above the floor, in meters. */
    public static double CAMERA_HEIGHT_M = Constants.Vision.CAMERA_HEIGHT_M;

    /** Height of the goal AprilTag's center above the floor, in meters. */
    public static double GOAL_TAG_HEIGHT_M = Constants.Vision.GOAL_TAG_HEIGHT_M;

    /** Upward tilt of the camera from horizontal, in degrees. */
    public static double CAMERA_MOUNT_ANGLE_DEG = Constants.Vision.CAMERA_MOUNT_ANGLE_DEG;
}
