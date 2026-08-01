package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
@TeleOp(name = "2WheelTest")
public class DriveTestOp extends OpMode {
    private Hardware hardware;
    DcMotorEx wheel1;
    DcMotorEx wheel2;

    @Override
    public void init(){
        hardware = new Hardware();
        hardware.initTwoWheels(hardwareMap);
        wheel1 = hardware.wheel1;
        wheel2 = hardware.wheel2;
    }

    @Override
    public void loop(){
        if(gamepad1.dpad_down){
            wheel1.setPower(-1);
            wheel2.setPower(-1);
        } else if(gamepad1.dpad_up){
            wheel1.setPower(1);
            wheel2.setPower(1);
        } else {
            wheel1.setPower(0);
            wheel2.setPower(0);
        }


    }


}
