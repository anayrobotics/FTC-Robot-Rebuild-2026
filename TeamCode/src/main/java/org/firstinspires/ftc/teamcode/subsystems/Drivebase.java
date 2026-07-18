package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.localization.NavXIMU;

public class Drivebase {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    // Heading from the navX2 (same source PedroPathing uses for pose).
    private final NavXIMU heading;

    public Drivebase(Hardware hardware){
        frontLeft = hardware.frontLeft;
        frontRight = hardware.frontRight;
        backLeft = hardware.backLeft;
        backRight = hardware.backRight;
        heading = hardware.navxImu;
    }

    public void drive(double axial, double lateral, double yaw){
        double flPower = axial + lateral + yaw;
        double frPower = axial - lateral - yaw;
        double blPower = axial - lateral + yaw;
        double brPower = axial + lateral - yaw;

        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower),
                        Math.max(Math.abs(blPower), Math.abs(brPower)))));

        frontLeft.setPower(flPower / max);
        frontRight.setPower(frPower / max);
        backLeft.setPower(blPower / max);
        backRight.setPower(brPower / max);
    }

    // Reads the gamepad sticks and drives the robot. Left stick translates, right
    // stick x turns. Pass fieldCentric = true to drive relative to the field.
    public void driveWithGamepad(Gamepad gamepad, boolean fieldCentric){
        double axial = deadzone(-gamepad.left_stick_y);
        double lateral = deadzone(gamepad.left_stick_x);
        double yaw = deadzone(gamepad.right_stick_x);

        if (fieldCentric) {
            driveFieldCentric(axial, lateral, yaw);
        } else {
            drive(axial, lateral, yaw);
        }
    }

    private static double deadzone(double value){
        return Math.abs(value) < Constants.Drive.DEADZONE ? 0.0 : value;
    }

    public void driveFieldCentric(double axial, double lateral, double yaw){
        double heading = getHeading();
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double rotatedLateral = lateral * cos - axial * sin;
        double rotatedAxial = lateral * sin + axial * cos;

        drive(rotatedAxial, rotatedLateral, yaw);
    }

    public double getHeading(){
        return heading.getHeading();
    }

    public void resetHeading(){
        heading.resetYaw();
    }

    public void stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
