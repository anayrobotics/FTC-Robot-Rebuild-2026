package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Live-tunable stopper servo positions, editable from the Panels dashboard while
 * the robot is running (no re-deploy).
 *
 * <p>Fields must be {@code public static} and <b>non-final</b> for Panels to
 * expose them. They are seeded from {@link Constants.Stopper} so that class stays
 * the source of the compile-time defaults; once you dial values in on Panels,
 * copy the good numbers back into {@link Constants.Stopper}.
 *
 * <p>{@link org.firstinspires.ftc.teamcode.subsystems.Stopper} reads these every
 * loop, so nudging a position on Panels moves the gate immediately if it's
 * currently sitting at that state.
 */
@Configurable
public class StopperTuning {
    // Where the gate holds the ball back, and where it clears the path.
    public static double BLOCKING_POSITION = Constants.Stopper.BLOCKING_POSITION;
    public static double OPEN_POSITION = Constants.Stopper.OPEN_POSITION;

    // Seconds allowed for the servo to travel between the two positions.
    public static double TRAVEL_TIME_S = Constants.Stopper.TRAVEL_TIME_S;
}
