package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.localization.NavXIMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Every device on the robot, pulled out of the hardware map in one place.
 *
 * <h2>Full init vs. per-group init</h2>
 * {@link #init(HardwareMap)} brings up everything and is what the match OpModes
 * use. It throws if ANY device is missing from the Robot Configuration, which is
 * correct for a match — you do not want to drive out with half a robot.
 *
 * <p>That is the wrong behaviour on the bench, though: a hood servo that isn't
 * plugged in yet would stop you testing the drivebase. So each group also has
 * its own {@code initX} method, and the test OpModes call only the ones they
 * need. {@link #scan(HardwareMap)} goes further and probes every device without
 * throwing at all, which is how the Hardware Scan test reports what's missing.
 */
public class Hardware {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intake;
    public DcMotorEx indexer;

    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;

    // Positional Axon servo that rotates the turret. Its programmed internal
    // limits protect the wiring; Java commands standard Servo positions.
    public Servo turret;

    // Positional servo that tilts the shooter hood (launch angle).
    public Servo hood;

    // Two-position gate servo just before the flywheel: holds the next ball back
    // until we're ready to shoot.
    public Servo stopper;

    // Limelight 3A smart camera (AprilTag targeting).
    public Limelight3A limelight;

    // Gyro / orientation sensor built into the REV Control Hub. Nothing reads it
    // — heading comes from the navX below — so it is initialized best-effort and
    // a failure here never takes an OpMode down.
    public IMU imu;

    // navX2-Micro: the robot's heading source for both drive and PedroPathing.
    public final NavXIMU navxImu = new NavXIMU();

    /**
     * Brings up every device. Throws if anything is missing from the Robot
     * Configuration — which is what the match OpModes want.
     */
    public void init(HardwareMap hw){
        initDrive(hw);
        initIntake(hw);
        initIndexer(hw);
        initFlywheel(hw);
        initTurret(hw);
        initHood(hw);
        initStopper(hw);
        initLimelight(hw);
        initImu(hw);
    }

    public void initDrive(HardwareMap hw){
        frontLeft = hw.get(DcMotorEx.class, Constants.Drive.FRONT_LEFT);
        frontRight = hw.get(DcMotorEx.class, Constants.Drive.FRONT_RIGHT);
        backLeft = hw.get(DcMotorEx.class, Constants.Drive.BACK_LEFT);
        backRight = hw.get(DcMotorEx.class, Constants.Drive.BACK_RIGHT);

        frontLeft.setDirection(Constants.Drive.LEFT_DIRECTION);
        backLeft.setDirection(Constants.Drive.LEFT_DIRECTION);
        frontRight.setDirection(Constants.Drive.RIGHT_DIRECTION);
        backRight.setDirection(Constants.Drive.RIGHT_DIRECTION);

        setZeroPowerBehavior(Constants.Drive.ZERO_POWER_BEHAVIOR);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(Constants.Drive.RUN_MODE);
    }

    public void initIntake(HardwareMap hw){
        intake = hw.get(DcMotorEx.class, Constants.Intake.MOTOR);
        intake.setDirection(Constants.Intake.DIRECTION);
    }

    public void initIndexer(HardwareMap hw){
        indexer = hw.get(DcMotorEx.class, Constants.Indexer.MOTOR);
        indexer.setDirection(Constants.Indexer.DIRECTION);
    }

    public void initFlywheel(HardwareMap hw){
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
    }

    public void initTurret(HardwareMap hw){
        turret = hw.get(Servo.class, Constants.Turret.SERVO);
        turret.setDirection(Constants.Turret.DIRECTION);
        // Do not command a position during INIT. The first active loop either
        // begins vision tracking or explicitly returns the turret to neutral.
    }

    public void initHood(HardwareMap hw){
        hood = hw.get(Servo.class, Constants.Hood.SERVO);
        hood.setDirection(Constants.Hood.DIRECTION);
        // Deliberately NOT commanded to a position here. The hub powers servos
        // up with PWM disabled, and the first setPosition() silently re-enables
        // it (LynxServoController auto-enables on every pulse-width write), so
        // commanding a position at init energizes the servo the instant you
        // press INIT — before anyone can check the linkage. If DEFAULT_POSITION
        // is past the hood's mechanical stop, it then sits there at full torque
        // for however long you spend on the INIT screen, which is exactly how a
        // servo cooks itself. Leaving it limp means you can back-drive the hood
        // by hand to see where it is. Hood.periodic() takes it from PLAY.
    }

    public void initStopper(HardwareMap hw){
        stopper = hw.get(Servo.class, Constants.Stopper.SERVO);
        stopper.setDirection(Constants.Stopper.DIRECTION);
        // Park the gate CLOSED at init so a preloaded ball can't sit against the
        // flywheel before the match starts.
        stopper.setPosition(Constants.Stopper.BLOCKING_POSITION);
    }

    public void initLimelight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class, Constants.Vision.LIMELIGHT);
        limelight.pipelineSwitch(Constants.Vision.PIPELINE);
        // Begin polling for results. Without start(), getLatestResult() is null.
        limelight.start();
    }

    /**
     * Heading sensors. The navX is the one that matters; the hub IMU is
     * initialized best-effort because nothing reads it and a hub-IMU fault
     * shouldn't cost us a match.
     */
    public void initImu(HardwareMap hw){
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                Constants.Imu.LOGO_DIRECTION,
                Constants.Imu.USB_DIRECTION);
        try {
            imu = hw.get(IMU.class, Constants.Imu.NAME);
            imu.initialize(new IMU.Parameters(hubOrientation));
            imu.resetYaw();
        } catch (RuntimeException ignored) {
            imu = null;
        }

        initNavx(hw);
    }

    /**
     * Brings up the navX (waiting out its power-on calibration) and zeroes it so
     * "forward at init" is heading 0.
     */
    public void initNavx(HardwareMap hw){
        navxImu.initialize(hw, Constants.Imu.NAVX, new RevHubOrientationOnRobot(
                Constants.Imu.LOGO_DIRECTION, Constants.Imu.USB_DIRECTION));
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

    // ------------------------------------------------------------------
    // Config scan — used by the Hardware Scan test to report what's actually
    // in the Robot Configuration before anything tries to use it.
    // ------------------------------------------------------------------

    /** One device's presence in the Robot Configuration. */
    public static final class DeviceCheck {
        public final String group;
        public final String configName;
        public final String type;
        public final boolean present;
        /** Null when present; otherwise why the lookup failed. */
        public final String problem;

        DeviceCheck(String group, String configName, String type,
                    boolean present, String problem) {
            this.group = group;
            this.configName = configName;
            this.type = type;
            this.present = present;
            this.problem = problem;
        }
    }

    /**
     * Looks up every device this robot expects, catching failures instead of
     * throwing. Nothing is configured, started, or moved — this only answers
     * "is it in the Robot Configuration under the right name and type?".
     */
    public static List<DeviceCheck> scan(HardwareMap hw){
        List<DeviceCheck> results = new ArrayList<>();

        check(results, hw, "Drive", Constants.Drive.FRONT_LEFT, DcMotorEx.class);
        check(results, hw, "Drive", Constants.Drive.FRONT_RIGHT, DcMotorEx.class);
        check(results, hw, "Drive", Constants.Drive.BACK_LEFT, DcMotorEx.class);
        check(results, hw, "Drive", Constants.Drive.BACK_RIGHT, DcMotorEx.class);

        check(results, hw, "Ball path", Constants.Intake.MOTOR, DcMotorEx.class);
        check(results, hw, "Ball path", Constants.Indexer.MOTOR, DcMotorEx.class);
        check(results, hw, "Shooter", Constants.Flywheel.LEFT_MOTOR, DcMotorEx.class);
        check(results, hw, "Shooter", Constants.Flywheel.RIGHT_MOTOR, DcMotorEx.class);

        check(results, hw, "Turret", Constants.Turret.SERVO, Servo.class);
        check(results, hw, "Shooter", Constants.Hood.SERVO, Servo.class);
        check(results, hw, "Shooter", Constants.Stopper.SERVO, Servo.class);

        check(results, hw, "Vision", Constants.Vision.LIMELIGHT, Limelight3A.class);

        check(results, hw, "Heading", Constants.Imu.NAVX, NavxMicroNavigationSensor.class);
        check(results, hw, "Heading", Constants.Imu.NAME, IMU.class);

        return results;
    }

    private static void check(List<DeviceCheck> into, HardwareMap hw, String group,
                              String name, Class<?> type){
        try {
            hw.get(type, name);
            into.add(new DeviceCheck(group, name, type.getSimpleName(), true, null));
        } catch (RuntimeException e) {
            String problem = e.getMessage() == null ? e.toString() : e.getMessage();
            // The SDK's message is a paragraph; the first line is the useful bit.
            int newline = problem.indexOf('\n');
            if (newline > 0) {
                problem = problem.substring(0, newline);
            }
            into.add(new DeviceCheck(group, name, type.getSimpleName(), false, problem));
        }
    }
}
