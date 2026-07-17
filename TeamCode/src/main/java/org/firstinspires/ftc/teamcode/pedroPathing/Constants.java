package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.Drive;

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
    // Localizer: read the four drive-motor encoders. No extra odometry
    // hardware needed, but it is the least accurate localizer — tune the
    // ticks-to-inches multipliers and track dimensions before trusting it.
    // ------------------------------------------------------------------
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName(Drive.FRONT_LEFT)
            .leftRearMotorName(Drive.BACK_LEFT)
            .rightFrontMotorName(Drive.FRONT_RIGHT)
            .rightRearMotorName(Drive.BACK_RIGHT)
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            // TODO tune: run Forward/Lateral/Turn tuners and paste results.
            .forwardTicksToInches(1.0)
            .strafeTicksToInches(1.0)
            .turnTicksToInches(1.0)
            // TODO tune: measure the robot's track width/length in inches.
            .robotWidth(12.0)
            .robotLength(12.0);

    // Global motion constraints for path following.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /** Builds a fully-configured {@link Follower} for the current OpMode. */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
