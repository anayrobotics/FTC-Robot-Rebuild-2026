package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

// State-based indexer. Commands set the state; periodic() applies it to the motor.
public class Indexer implements Subsystem {
    public enum State {
        IDLE,
        FEEDING,
        REVERSING
    }

    private final DcMotorEx motor;
    private State state = State.IDLE;

    public Indexer(Hardware hardware){
        motor = hardware.indexer;
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
            case FEEDING:
                motor.setPower(Constants.Indexer.SPEED);
                break;
            case REVERSING:
                motor.setPower(-Constants.Indexer.SPEED);
                break;
            case IDLE:
            default:
                motor.setPower(0);
                break;
        }
    }
}
