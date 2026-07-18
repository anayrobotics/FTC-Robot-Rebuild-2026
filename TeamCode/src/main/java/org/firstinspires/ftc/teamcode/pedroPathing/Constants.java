package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
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
 * and {@code Hardware}/{@code Drivebase}), and uses the four drive-motor
 * encoders for localization since the robot has no dedicated odometry pods.
 *
 * <p><b>TUNING REQUIRED.</b> The values marked {@code TODO tune} below are
 * PedroPathing defaults / rough guesses. Paths will only be accurate after you
 * run the PedroPathing tuning OpModes on the real robot and paste the measured
 * numbers back here. See https://pedropathing.com/docs/pathing/tuning .
 */
public class Constants {

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
    // Localizer: two dead-wheel odometry pods for translation + the navX2 for
    // heading (via NavXIMU). Far more accurate than drive encoders, and the
    // heading matches the field-centric drive since both read the same navX.
    //
    // Pod encoders plug into (unused) motor encoder ports; the names below are
    // the CONFIG NAMES of whatever ports the pods are wired to. Directions,
    // pod offsets, and ticks-to-inches all need on-robot tuning.
    // ------------------------------------------------------------------
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
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
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
