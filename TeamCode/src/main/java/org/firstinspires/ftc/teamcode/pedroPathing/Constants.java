package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Drive;
import org.firstinspires.ftc.teamcode.Constants.Imu;
import org.firstinspires.ftc.teamcode.localization.NavXIMU;

/**
 * PedroPathing configuration for our mecanum robot.
 *
 * <p>This wires the follower to the SAME motor configuration names and
 * directions our TeleOp already uses (see {@link org.firstinspires.ftc.teamcode.Constants.Drive}
 * and {@code Hardware}/{@code Drivebase}).
 *
 * <h2>Which localizer</h2>
 * {@link #USE_ODOMETRY_PODS} picks between the two. It is <b>false</b> today
 * because the robot has no pods fitted: the two-wheel localizer would ask the
 * hardware map for {@code forwardOdo} and {@code strafeOdo}, not find them, and
 * throw — which takes down every OpMode that builds a follower (Drive To Pose,
 * Pedro Auto, and PedroPathing's own Tuning menu) at init.
 *
 * <p>With it false we localize off the four drive-motor encoders instead. That
 * needs no hardware we don't already have, but it is materially worse: wheel
 * slip goes straight into the pose estimate, and heading is derived from the
 * left/right wheel difference rather than read off the navX, so it drifts.
 * Good enough to bring drive-to-pose up on the bench; not good enough for a
 * real autonomous. Flip the flag the day the pods are on the robot.
 *
 * <p><b>TUNING REQUIRED.</b> The values marked {@code TODO tune} below are
 * PedroPathing defaults / rough guesses. Paths will only be accurate after you
 * run the PedroPathing tuning OpModes on the real robot and paste the measured
 * numbers back here. See https://pedropathing.com/docs/pathing/tuning .
 */
public class Constants {

    /** True once the dead-wheel odometry pods are fitted and configured. */
    public static final boolean USE_ODOMETRY_PODS = false;

    // ------------------------------------------------------------------
    // Follower (mass + zero-power decel used by the path controller).
    // ------------------------------------------------------------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            // TODO tune: robot mass in kg (default 10.65). Measure the real robot.
            .mass(10.65);

    // ------------------------------------------------------------------
    // Mecanum drivetrain: config names + directions mirror Drivebase.
    // Left side is REVERSE so positive power drives forward.
    // ------------------------------------------------------------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(Drive.FRONT_LEFT)
            .leftRearMotorName(Drive.BACK_LEFT)
            .rightFrontMotorName(Drive.FRONT_RIGHT)
            .rightRearMotorName(Drive.BACK_RIGHT)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // TODO tune: max wheel velocities (in/s) from the velocity tuners.
            .xVelocity(57.0)
            .yVelocity(45.0);

    // ------------------------------------------------------------------
    // Fallback localizer: the four drive-motor encoders. In use while
    // USE_ODOMETRY_PODS is false.
    // ------------------------------------------------------------------
    public static DriveEncoderConstants driveEncoderLocalizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName(Drive.FRONT_LEFT)
            .leftRearMotorName(Drive.BACK_LEFT)
            .rightFrontMotorName(Drive.FRONT_RIGHT)
            .rightRearMotorName(Drive.BACK_RIGHT)
            // Which way each ENCODER counts — a different question from which way
            // the motor is wired. If pushing the robot forward makes the reported
            // X go down in the Localization Test, flip all four.
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            // TODO tune: run PedroPathing's Forward / Lateral / Turn tuners.
            .forwardTicksToInches(0.0029)
            .strafeTicksToInches(0.0029)
            .turnTicksToInches(0.0029)
            // TODO measure: track width and wheelbase, in inches.
            .robotWidth(14.0)
            .robotLength(14.0);

    // ------------------------------------------------------------------
    // Preferred localizer: two dead-wheel odometry pods for translation + the
    // navX2 for heading (via NavXIMU). Far more accurate than drive encoders,
    // and the heading matches field-centric drive since both read the same navX.
    //
    // NOT IN USE until USE_ODOMETRY_PODS is set true.
    //
    // Pod encoders plug into (unused) motor encoder ports; the names below are
    // the CONFIG NAMES of whatever ports the pods are wired to. Directions,
    // pod offsets, and ticks-to-inches all need on-robot tuning.
    // ------------------------------------------------------------------
    public static TwoWheelConstants podLocalizerConstants = new TwoWheelConstants()
            // TODO set: config names of the two ports the odometry pods plug into.
            .forwardEncoder_HardwareMapName("forwardOdo")
            .strafeEncoder_HardwareMapName("strafeOdo")
            // TODO tune: flip a direction if that axis reads backwards in the
            // Localization Test (push robot forward -> forward value must rise).
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            // TODO measure (inches, robot-centric from the tracking center):
            //   forwardPodY = left/right offset of the FORWARD pod (+left)
            //   strafePodX  = fwd/back offset of the STRAFE pod (+forward)
            .forwardPodY(1.0)
            .strafePodX(-2.5)
            // TODO tune: run the Forward/Lateral tuners and paste the results.
            .forwardTicksToInches(0.001989436789)
            .strafeTicksToInches(0.001989436789)
            // Heading from the navX2, not the hub IMU. The name is passed to
            // NavXIMU.initialize(); orientation is unused by the navX adapter.
            .IMU_HardwareMapName(Imu.NAVX)
            .customIMU(new NavXIMU());

    // Global motion constraints for path following.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /** Builds a fully-configured {@link Follower} for the current OpMode. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerBuilder builder = new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints);

        if (USE_ODOMETRY_PODS) {
            builder.twoWheelLocalizer(podLocalizerConstants);
        } else {
            builder.driveEncoderLocalizer(driveEncoderLocalizerConstants);
        }

        return builder.build();
    }

    /** Human-readable name of the localizer currently in use, for telemetry. */
    public static String localizerName() {
        return USE_ODOMETRY_PODS ? "two-wheel odometry pods + navX"
                                 : "drive encoders (no pods — expect drift)";
    }
}
