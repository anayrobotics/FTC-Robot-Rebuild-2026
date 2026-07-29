package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.localization.NavXIMU;

public class Hardware {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intake;
    public DcMotorEx indexer;

    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;

    // Positional servo that tilts the shooter hood (launch angle).
    public Servo hood;

    // Gyro / orientation sensor (built into the REV Control/Expansion Hub).
    // Still initialized, but heading now comes from the navX below.
    public IMU imu;

    // navX2-Micro: the robot's heading source for both drive and PedroPathing.
    public final NavXIMU navxImu = new NavXIMU();

    // Minimal init for isolated shooter bring-up. Grabs ONLY the two flywheel
    // motors and the hood servo, so the OpMode runs on a Control Hub that has
    // just those three devices in its config (no drivetrain, intake, indexer,
    // Limelight, or IMU). The full init() below would throw on the first
    // missing device.
    //
    // Use this OR init(), never both. Leaves every other field null, so only
    // run code that touches the flywheel or hood after calling it. The flywheel
    // is set up for OPEN-LOOP power (no velocity PID): float when idle so it
    // coasts, and RUN_WITHOUT_ENCODER so raw power goes straight to the motors.
    public void initFlywheelAndHood(HardwareMap hw){
        flywheelLeft = hw.get(DcMotorEx.class, Constants.Flywheel.LEFT_MOTOR);
        flywheelRight = hw.get(DcMotorEx.class, Constants.Flywheel.RIGHT_MOTOR);
        flywheelLeft.setDirection(Constants.Flywheel.LEFT_DIRECTION);
        flywheelRight.setDirection(Constants.Flywheel.RIGHT_DIRECTION);
        // Coast, not brake — a flywheel should spin down freely.
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Raw power in, no SDK velocity loop. getVelocity() still reads the
        // encoder (if one is wired) so we can display the resulting RPM.
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hw.get(Servo.class, Constants.Hood.SERVO);
        hood.setDirection(Constants.Hood.DIRECTION);
        // Start stowed so the hood doesn't slam to a random angle on init.
        hood.setPosition(Constants.Hood.DEFAULT_POSITION);
    }

    public void init(HardwareMap hw){
        frontLeft = hw.get(DcMotorEx.class, Constants.Drive.FRONT_LEFT);
        frontRight = hw.get(DcMotorEx.class, Constants.Drive.FRONT_RIGHT);
        backLeft = hw.get(DcMotorEx.class, Constants.Drive.BACK_LEFT);
        backRight = hw.get(DcMotorEx.class, Constants.Drive.BACK_RIGHT);

        frontLeft.setDirection(Constants.Drive.LEFT_DIRECTION);
        backLeft.setDirection(Constants.Drive.LEFT_DIRECTION);
        frontRight.setDirection(Constants.Drive.RIGHT_DIRECTION);
        backRight.setDirection(Constants.Drive.RIGHT_DIRECTION);

        intake = hw.get(DcMotorEx.class, Constants.Intake.MOTOR);
        intake.setDirection(Constants.Intake.DIRECTION);

        indexer = hw.get(DcMotorEx.class, Constants.Indexer.MOTOR);
        indexer.setDirection(Constants.Indexer.DIRECTION);

        flywheelLeft = hw.get(DcMotorEx.class, Constants.Flywheel.LEFT_MOTOR);
        flywheelRight = hw.get(DcMotorEx.class, Constants.Flywheel.RIGHT_MOTOR);
        flywheelLeft.setDirection(Constants.Flywheel.LEFT_DIRECTION);
        flywheelRight.setDirection(Constants.Flywheel.RIGHT_DIRECTION);
        // Coast, not brake — a flywheel should spin down freely.
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // We run our own velocity PID, so let the motors take raw power.
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hood = hw.get(Servo.class, Constants.Hood.SERVO);
        hood.setDirection(Constants.Hood.DIRECTION);
        // Start stowed so the hood doesn't slam to a random angle on init.
        hood.setPosition(Constants.Hood.DEFAULT_POSITION);

        setZeroPowerBehavior(Constants.Drive.ZERO_POWER_BEHAVIOR);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                Constants.Imu.LOGO_DIRECTION,
                Constants.Imu.USB_DIRECTION);
        imu = hw.get(IMU.class, Constants.Imu.NAME);
        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();

        // Bring up the navX (waits out its power-on calibration) and zero it so
        // "forward at init" is heading 0, matching the old hub-IMU behavior.
        navxImu.initialize(hw, Constants.Imu.NAVX, hubOrientation);
        navxImu.resetYaw();
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setMode(DcMotor.RunMode mode){
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}
