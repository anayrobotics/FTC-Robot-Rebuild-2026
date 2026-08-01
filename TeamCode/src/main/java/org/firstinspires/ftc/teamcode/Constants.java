package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Constants {

    public static final class Drive {
        // Hardware map config names.
        public static final String FRONT_LEFT = "frontLeftDrive";
        public static final String FRONT_RIGHT = "frontRightDrive";
        public static final String BACK_LEFT = "backLeftDrive";
        public static final String BACK_RIGHT = "backRightDrive";

        // Right side reversed so positive power drives forward.
        public static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.REVERSE;

        public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

        // RUN_USING_ENCODER gives smoother, more repeatable driving — but ONLY if
        // all four drive motors have their encoder cables plugged in. With a
        // cable missing, that motor reads zero velocity, the hub's internal
        // velocity loop pins it to full power, and the robot veers hard.
        //
        // If the Drive Motor Check test shows a motor whose ticks never change,
        // either plug the encoder in or drop this to RUN_WITHOUT_ENCODER.
        public static final DcMotor.RunMode RUN_MODE = DcMotor.RunMode.RUN_USING_ENCODER;

        //stick input deadzone
        public static final double DEADZONE = 0.05;
    }

    public static final class Intake {
        public static final String MOTOR = "intake";
        public static final DcMotor.Direction DIRECTION = DcMotor.Direction.FORWARD;
        public static final double SPEED = 1.0;
    }

    public static final class Indexer {
        public static final String MOTOR = "indexer";
        public static final DcMotor.Direction DIRECTION = DcMotor.Direction.REVERSE;
        public static final double SPEED = 1.0;
    }

    public static final class Flywheel {
        public static final String LEFT_MOTOR = "flywheelLeft";
        public static final String RIGHT_MOTOR = "flywheelRight";

        // The two motors face opposite ways, so one is reversed to spin the wheel together.
        public static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

        // Encoder counts per revolution of the flywheel shaft. 28 = bare REV/goBILDA
        // 6000 rpm motor (no gearbox). Change if your flywheel is geared.
        public static final double TICKS_PER_REV = 28.0;

        // Free speed of the flywheel shaft; used to clamp targets.
        public static final double MAX_RPM = 6000.0;

        // Velocity PIDF gains. kF is the dominant term (feedforward = kF * targetRpm),
        // P/I/D correct the remaining error. Tune kF first, then kP, then kD.
        public static final double kP = 0.0005; //0.0003
        public static final double kI = 0.0;
        public static final double kD = 0.00003;
        public static final double kF = 1.0 / MAX_RPM;

        // Considered "at target", and therefore ready to START a shot, within
        // this many RPM.
        public static final double RPM_TOLERANCE = 25.0;

        // The band that KEEPS a burst feeding once it has started. Much wider,
        // and it has to be: putting a ball through the wheel drops it by a
        // couple of hundred RPM, which is far outside RPM_TOLERANCE. Gate the
        // feed on the start band and every single shot slams the stopper shut
        // and pays its travel time again on the way back — you get a fraction
        // of the fire rate for no accuracy gain, because the wheel has already
        // recovered most of the way by the time the next ball reaches it.
        public static final double RPM_KEEP_TOLERANCE = 400.0;

        // A setpoint change smaller than this is auto-ranging jitter, not a new
        // shot, so the PID keeps its history. Separate from RPM_TOLERANCE so
        // tightening the at-speed band doesn't also make the loop reset more.
        public static final double SETPOINT_JUMP_RPM = 300.0;

        // Preset shooting speed (fallback when no target is visible for ranging).
        public static final double SHOOT_RPM = 3500.0;

        // Distance-based shooting table for auto-ranging off the Limelight.
        // Parallel arrays: at RANGE_DISTANCES_M[i] meters from the goal, spin
        // RANGE_RPMS[i]. Values between points are linearly interpolated; outside
        // the range they clamp to the nearest end. These are STARTING GUESSES —
        // measure real distances and log the RPM that scores, then edit these.
        // Must be the same length, and distances must be strictly increasing.
        public static final double[] RANGE_DISTANCES_M = {1.0, 2.0, 3.0, 4.0};
        public static final double[] RANGE_RPMS        = {2600, 3100, 3600, 4200};
    }

    public static final class Turret {
        // POSITIONAL servo that rotates the turret. It is commanded to an angle
        // and holds it — the same kind of device as the hood and stopper below,
        // just geared to swing the whole shooter.
        //
        // The turret therefore has no idea where it is beyond what we last told
        // it, and that is fine, because a positional servo can only ever be
        // where it was told. There is no feedback wire, no winding to unwrap and
        // no travel accumulator: the command IS the position. What replaces all
        // of that is MIN_ANGLE_DEG / MAX_ANGLE_DEG below, which every command is
        // clamped to.
        //
        // The one thing this costs: nothing surveys the turret at startup. If it
        // is physically off-centre when the OpMode begins, the first command
        // snaps it across. START EVERY MATCH WITH THE TURRET POINTED STRAIGHT
        // AHEAD.
        public static final String SERVO = "turret";

        // Which way the turret swings for an increasing servo position. Flip to
        // REVERSE if positive angles below come out on the wrong side; that
        // keeps the geometry sign here and leaves INVERT_OUTPUT to be purely
        // about the camera.
        public static final com.qualcomm.robotcore.hardware.Servo.Direction DIRECTION =
                com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

        // MEASURE THIS. Degrees of TURRET rotation across the servo's full
        // [0, 1] travel — the servo's programmed range, divided by any gear
        // reduction between it and the turret.
        //
        // Everything in this class is expressed in degrees and converted through
        // this one number, so if it is wrong every angle is wrong by the same
        // factor: the turret will consistently under- or over-shoot and no
        // amount of gain tuning will fix it. Run test 4a, command a known angle,
        // and measure what the turret actually did with a protractor.
        public static final double SERVO_RANGE_DEG = 355.0;

        // MEASURE THIS TOO. Servo position, in [0, 1], at which the turret
        // points straight ahead. Centred by default so the turret has equal
        // travel either way. Run test 4a, jog until it is dead centre, and copy
        // the reported position here.
        public static final double ORIGIN_POSITION = 0.5;

        // Software travel limits, in degrees off the origin. Every command is
        // clamped to these, in every state, so a bad gain or a goal behind the
        // robot can never drive the turret into its own hard stop.
        //
        // TIGHTEN THESE to the real mechanical range once you have found it —
        // they start deliberately narrower than SERVO_RANGE_DEG allows.
        public static final double MIN_ANGLE_DEG = -150.0;
        public static final double MAX_ANGLE_DEG = 150.0;

        // Aim loop. Input is the Limelight's horizontal error (tx, degrees);
        // output is a turret SLEW RATE in degrees per second, which the subsystem
        // integrates into the position it commands.
        //
        // Rate rather than position on purpose. Commanding "current angle + tx"
        // outright looks like the obvious move for a positional servo and is a
        // trap: the servo needs a good fraction of a second to get there, tx
        // keeps reporting the old error the whole way, and the loop piles
        // correction on correction until it slams past centre. Asking for a rate
        // the servo can actually keep up with makes the commanded angle track
        // the real one, which is the whole basis for trusting it as position.
        //
        // kI stays 0 — a turret that briefly loses sight of the tag would wind
        // the integral up while blind and then slam. kF stays 0 because the
        // setpoint is tx = 0.
        public static final double kP = 6.0;
        public static final double kI = 0.0;
        public static final double kD = 0.10;
        public static final double kF = 0.0;

        // If the turret swings AWAY from the target, flip this. Purely about how
        // the camera is mounted relative to the turret's rotation — the servo's
        // own geometry sign lives in DIRECTION above.
        public static final boolean INVERT_OUTPUT = false;

        // MEASURE THIS. Ceiling on the commanded slew, degrees per second, and
        // the most important number here after SERVO_RANGE_DEG.
        //
        // It must be at or BELOW the servo's real loaded speed. Set it higher
        // and the command runs away from where the turret actually is, which
        // breaks the assumption everything else rests on: isAtOrigin() reports
        // home while it is still swinging, and the limits stop meaning anything.
        // Take the servo's no-load spec (deg/sec), scale it by the gearing, and
        // knock 30% off for load.
        public static final double MAX_SLEW_DEG_PER_S = 180.0;

        // Park speed for RETURN_TO_ORIGIN. Slower than the aim: this one runs
        // unattended while the driver is busy somewhere else.
        public static final double MAX_RETURN_DEG_PER_S = 120.0;

        // Inside this many degrees of the origin, we count as parked.
        public static final double RETURN_TOLERANCE_DEG = 2.0;

        // Inside this many degrees of the goal we consider ourselves aimed and
        // stop commanding, which kills the hunting you'd otherwise get as the
        // loop chases sub-pixel noise around centre.
        public static final double AIM_TOLERANCE_DEG = 1.0;

        // The band that KEEPS a burst feeding once it has started, the same idea
        // as Flywheel.RPM_KEEP_TOLERANCE. Because the turret stops commanding
        // the instant it's inside AIM_TOLERANCE_DEG, the aim drifts back out,
        // the turret nudges, and the lock flag flickers on and off around centre
        // even with a stationary robot. That flicker is not a real loss of aim,
        // and stopping the feed for it costs a shot every time.
        public static final double KEEP_AIM_TOLERANCE_DEG = 3.0;

        // Speed of a manual dpad nudge in TeleOp, degrees per second.
        // Deliberately slower than the aim: this is the driver hunting for a
        // target by eye, so it should creep.
        public static final double MANUAL_NUDGE_DEG_PER_S = 45.0;
    }

    public static final class Hood {
        // Positional servo that tilts the shooter hood, setting the ball's
        // launch angle. Like the turret above, a standard Servo commanded to a
        // repeatable position in [0, 1] and held there.
        //
        // Convention used below: a HIGHER position raises the hood to a steeper
        // launch angle (shorter, higher arc). If yours is geared the other way,
        // just swap DIRECTION or invert your preset/table values — nothing in
        // the code assumes a direction beyond "bigger number = the servo turns
        // one way".
        public static final String SERVO = "hood";
        public static final com.qualcomm.robotcore.hardware.Servo.Direction DIRECTION =
                com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

        // Safe travel band. The hood is clamped to this every loop so a bad
        // preset, a stale auto-range value, or a mis-scaled table can never
        // drive the linkage into a hard stop and strip the servo. TIGHTEN these
        // to the real mechanical limits once you've found them on the robot.
        public static final double MIN_POSITION = 0.00;
        public static final double MAX_POSITION = 0.4;

        // Where the hood sits at init and when the shooter is idle (stowed low).
        public static final double DEFAULT_POSITION = MIN_POSITION;

        // Manual presets the operator can snap to (dpad left/right in TeleOp)
        // when not auto-ranging: a close, flatter shot and a far, steeper shot.
        public static final double NEAR_PRESET = 0.30;
        public static final double FAR_PRESET = 0.70;

        // Distance-based hood table for auto-ranging off the Limelight, exactly
        // like Flywheel.RANGE_*. At RANGE_DISTANCES_M[i] meters from the goal,
        // set the hood to RANGE_POSITIONS[i]. Values between points are linearly
        // interpolated; outside the range they clamp to the nearest end. These
        // are STARTING GUESSES — shoot at known distances and log the position
        // that scores, then edit these. Both arrays must be the same length and
        // the distances strictly increasing.
        public static final double[] RANGE_DISTANCES_M = {1.0, 2.0, 3.0, 4.0};
        public static final double[] RANGE_POSITIONS   = {0.30, 0.45, 0.60, 0.70};
    }

    public static final class Stopper {
        // Positional servo gate sitting between the indexer and the flywheel. It
        // only ever lives at two positions: BLOCKING (holds the next ball back so
        // it can't touch the wheel) and OPEN (clears the path to fire). Keeping a
        // ball off the flywheel until we're aimed and up to speed is what stops
        // the wheel getting bogged down mid-spinup.
        public static final String SERVO = "stopper";
        public static final com.qualcomm.robotcore.hardware.Servo.Direction DIRECTION =
                com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

        // The two positions. These are STARTING GUESSES — jog the servo on the
        // real robot (Panels, via StopperTuning) and copy the values that
        // actually block and actually clear back into here.
        public static final double BLOCKING_POSITION = 0.25;
        public static final double OPEN_POSITION = 0.65;

        // How long the servo needs to physically travel between the two
        // positions, in seconds. Used to report when the gate has finished
        // moving — a servo has no position feedback, so this is just a timer.
        public static final double TRAVEL_TIME_S = 0.25;
    }

    public static final class DriveToPose {
        // Point-to-point drive controller: three PIDs run in the ROBOT frame and
        // drive the pose error (from PedroPathing's localizer) to zero.
        //
        // Forward and strafe are tuned separately because a mecanum strafes less
        // efficiently than it drives forward, so they usually want different
        // gains. Start them equal, then raise the strafe gains if sideways moves
        // lag. Errors are in INCHES (translation) and RADIANS (heading); outputs
        // are drive motor power.
        public static final double FORWARD_kP = 0.03;
        public static final double FORWARD_kI = 0.0;
        public static final double FORWARD_kD = 0.002;

        public static final double STRAFE_kP = 0.03;
        public static final double STRAFE_kI = 0.0;
        public static final double STRAFE_kD = 0.002;

        public static final double HEADING_kP = 0.8;
        public static final double HEADING_kI = 0.0;
        public static final double HEADING_kD = 0.05;

        // If an axis drives AWAY from the target (error grows, or the robot runs
        // off), flip that axis. The correct signs depend on motor directions and
        // how your field frame is oriented — verify each on the real robot.
        public static final boolean INVERT_FORWARD = false;
        public static final boolean INVERT_STRAFE = false;
        public static final boolean INVERT_HEADING = false;

        // Cap per-axis power so the robot eases into the target instead of
        // slamming toward it and overshooting.
        public static final double MAX_DRIVE_POWER = 0.6;
        public static final double MAX_TURN_POWER = 0.5;

        // Considered "at the target pose" once inside both bands.
        public static final double POSITION_TOLERANCE_IN = 1.0;
        public static final double HEADING_TOLERANCE_DEG = 2.0;
    }

    public static final class Vision {
        public static final String LIMELIGHT = "limelight";

        // Index of the AprilTag pipeline configured in the Limelight web UI.
        public static final int PIPELINE = 0;

        // DECODE (2025-2026) AprilTags, all in the 36h11 family.
        //   20 = BLUE goal          (aim target when on blue alliance)
        //   21 = obelisk motif GPP  (green-purple-purple)
        //   22 = obelisk motif PGP  (purple-green-purple)
        //   23 = obelisk motif PPG  (purple-purple-green)
        //   24 = RED goal           (aim target when on red alliance)
        public static final int BLUE_GOAL_TAG = 20;
        public static final int OBELISK_MOTIF_GPP = 21;
        public static final int OBELISK_MOTIF_PGP = 22;
        public static final int OBELISK_MOTIF_PPG = 23;
        public static final int RED_GOAL_TAG = 24;

        // Which goal the turret aims at by default. Change with the alliance
        // (or flip it live from the OpMode).
        public static final int DEFAULT_TARGET_TAG = RED_GOAL_TAG;

        // Camera geometry for estimating distance from the target's vertical
        // angle (ty). Measure these on the real robot/field, in meters/degrees.
        //   CAMERA_HEIGHT_M     : lens height above the floor
        //   GOAL_TAG_HEIGHT_M   : height of the goal AprilTag's center
        //   CAMERA_MOUNT_ANGLE  : upward tilt of the camera from horizontal
        public static final double CAMERA_HEIGHT_M = 0.30;
        public static final double GOAL_TAG_HEIGHT_M = 0.95;
        public static final double CAMERA_MOUNT_ANGLE_DEG = 20.0;
    }

    public static final class Imu {
        // Control Hub's built-in IMU (kept configured; heading now comes from navX).
        public static final String NAME = "imu";

        // Kauai Labs navX2-Micro (device name in the robot config). This is now the
        // single heading source for BOTH field-centric drive and PedroPathing pose,
        // read through the SDK's built-in NavxMicroNavigationSensor driver.
        public static final String NAVX = "navx";

        // Match to how the Control Hub is physically mounted.
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    }
}
