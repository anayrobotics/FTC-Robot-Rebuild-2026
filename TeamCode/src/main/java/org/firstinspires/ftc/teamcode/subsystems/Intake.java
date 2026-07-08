package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

// State-based intake. Commands set the state; periodic() applies it to the motor.
public class Intake implements Subsystem {
    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING
    }

    private final DcMotorEx motor;
    private State state = State.IDLE;

    public Intake(Hardware hardware){
        motor = hardware.intake;
    }

    public void setState(State state){
        this.state = state;
    }

    public State getState(){
        return state;
    }

    @Override
    public void periodic(){
        switch (state) {
            case INTAKING:
                motor.setPower(Constants.Intake.SPEED);
                break;
            case OUTTAKING:
                motor.setPower(-Constants.Intake.SPEED);
                break;
            case IDLE:
            default:
                motor.setPower(0);
                break;
        }
    }
}
