package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;

// State-based indexer. Commands set the state; periodic() applies it to the
// motors. Two motors (front and back) move the ball through the indexer; they
// are always driven together with the same signed power, so the two stages can
// never fight each other. Which way each one physically spins is set by its
// direction in Constants.Indexer, not by negating power here.
public class Indexer implements Subsystem {
    public enum State {
        IDLE,
        FEEDING,
        REVERSING,
        // Bring-up only: run one motor at a time to confirm each is wired to
        // the name it has in the config and spins the way FEEDING needs. Match
        // TeleOp never uses these — feeding always drives both together.
        FRONT_ONLY,
        BACK_ONLY
    }

    private final DcMotorEx frontMotor;
    private final DcMotorEx backMotor;
    private State state = State.IDLE;

    public Indexer(Hardware hardware){
        frontMotor = hardware.frontIndexer;
        backMotor = hardware.backIndexer;
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
                setPower(Constants.Indexer.SPEED);
                break;
            case REVERSING:
                setPower(-Constants.Indexer.SPEED);
                break;
            case FRONT_ONLY:
                frontMotor.setPower(Constants.Indexer.SPEED);
                backMotor.setPower(0);
                break;
            case BACK_ONLY:
                frontMotor.setPower(0);
                backMotor.setPower(Constants.Indexer.SPEED);
                break;
            case IDLE:
            default:
                setPower(0);
                break;
        }
    }

    // Single place both motors are commanded, so they always stay in lockstep.
    private void setPower(double power){
        frontMotor.setPower(power);
        backMotor.setPower(power);
    }
}
