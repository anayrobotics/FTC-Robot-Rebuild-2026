package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.rev.RevHubOrientationOnRobot;

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

        // Preset shooting speed.
        public static final double SHOOT_RPM = 3500.0;
    }

    public static final class Imu {
        public static final String NAME = "imu";

        // Match to how the Control Hub is physically mounted.
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    }
}
