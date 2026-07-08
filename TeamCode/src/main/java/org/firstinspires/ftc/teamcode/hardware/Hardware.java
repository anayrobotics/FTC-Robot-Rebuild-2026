package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Constants;

public class Hardware {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intake;
    public DcMotorEx indexer;

    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;

    // Gyro / orientation sensor (built into the REV Control/Expansion Hub).
    public IMU imu;

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

        setZeroPowerBehavior(Constants.Drive.ZERO_POWER_BEHAVIOR);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hw.get(IMU.class, Constants.Imu.NAME);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                Constants.Imu.LOGO_DIRECTION,
                Constants.Imu.USB_DIRECTION));
        imu.initialize(params);
        imu.resetYaw();
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
