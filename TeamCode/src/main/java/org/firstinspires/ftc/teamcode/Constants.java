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
        public static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

        public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;

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
        public static final DcMotor.Direction DIRECTION = DcMotor.Direction.FORWARD;
        public static final double SPEED = 1.0;
    }

    public static final class Flywheel {
        public static final String LEFT_MOTOR = "flywheelLeft";
        public static final String RIGHT_MOTOR = "flywheelRight";

        // The two motors face opposite ways, so one is reversed to spin the wheel together.
        public static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.REVERSE;

        // Encoder counts per revolution of the flywheel shaft. 28 = bare REV/goBILDA
        // 6000 rpm motor (no gearbox). Change if your flywheel is geared.
        public static final double TICKS_PER_REV = 28.0;

        // Free speed of the flywheel shaft; used to clamp targets.
        public static final double MAX_RPM = 6000.0;

        // Velocity PIDF gains. kF is the dominant term (feedforward = kF * targetRpm),
        // P/I/D correct the remaining error. Tune kF first, then kP, then kD.
        public static final double kP = 0.0003;
        public static final double kI = 0.0;
        public static final double kD = 0.00001;
        public static final double kF = 1.0 / MAX_RPM;

        // Considered "at target" within this many RPM.
        public static final double RPM_TOLERANCE = 75.0;

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
        // Continuous-rotation servo that spins the turret. A CRServo (not a
        // positional servo) is used so the aim loop can command a rotation
        // *speed* proportional to how far off target we are.
        //
        // REAL-LIFE WARNING: a CRServo has no position feedback and no built-in
        // travel limit, so nothing here stops the turret rotating past its
        // mechanical range and twisting the wiring. Add a hard mechanical stop
        // or a slip ring, and never leave it in AUTO_AIM without a valid target.
        public static final String SERVO = "turret";
        public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

        // Aim loop runs on the Limelight's horizontal error (tx, in degrees) and
        // drives it to zero. Output is servo power. No feedforward (kF) because
        // the setpoint is tx = 0, and no integral (kI) because a turret that can
        // briefly lose its target would wind the integral up and overshoot.
        public static final double kP = 0.020;
        public static final double kI = 0.0;
        public static final double kD = 0.0015;
        public static final double kF = 0.0;

        // If the turret drives AWAY from the target (runs to a hard stop or
        // oscillates and grows), flip this. The correct sign depends on which
        // way the servo is geared to the turret and how the camera is mounted.
        public static final boolean INVERT_OUTPUT = false;

        // Inside this many degrees we consider ourselves aimed and stop moving,
        // which kills the servo jitter you'd otherwise get right at center.
        public static final double AIM_TOLERANCE_DEG = 1.0;

        // Cap the aim speed so the turret slews smoothly instead of slamming.
        public static final double MAX_AIM_POWER = 0.6;

        // A CRServo below this power usually can't overcome its own stiction, so
        // when we do need to move we floor the command to at least this much.
        public static final double MIN_AIM_POWER = 0.05;
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
