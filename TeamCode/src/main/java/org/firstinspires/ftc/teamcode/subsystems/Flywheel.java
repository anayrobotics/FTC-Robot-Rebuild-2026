package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.PIDFController;

// Two-motor flywheel with closed-loop velocity control for exact RPM.
// Commands set a target RPM; periodic() runs the PIDF loop and drives both motors.
public class Flywheel implements Subsystem {
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final PIDFController controller =
            new PIDFController(Constants.Flywheel.kP, Constants.Flywheel.kI,
                    Constants.Flywheel.kD, Constants.Flywheel.kF);

    private double targetRpm = 0;
    private double currentRpm = 0;

    public Flywheel(Hardware hardware) {
        left = hardware.flywheelLeft;
        right = hardware.flywheelRight;
    }

    public void setTargetRpm(double rpm) {
        double clamped = Range.clip(rpm, 0, Constants.Flywheel.MAX_RPM);
        if (clamped != targetRpm) {
            controller.reset();
        }
        targetRpm = clamped;
    }

    public void stop() {
        setTargetRpm(0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    // Flywheel speed in RPM as of the last periodic() (read from the left encoder).
    public double getCurrentRpm() {
        return currentRpm;
    }

    public boolean atTargetRpm() {
        return targetRpm > 0
                && Math.abs(targetRpm - currentRpm) <= Constants.Flywheel.RPM_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Sample the encoder once per loop so the PID, atTargetRpm(), and telemetry
        // all use the same reading (and we only hit the hub once).
        currentRpm = left.getVelocity() / Constants.Flywheel.TICKS_PER_REV * 60.0;

        if (targetRpm <= 0) {
            // Let it coast to a stop rather than fighting the PID down to zero.
            left.setPower(0);
            right.setPower(0);
            return;
        }

        double power = Range.clip(controller.calculate(targetRpm, currentRpm), 0, 1);
        left.setPower(power);
        right.setPower(power);
    }
}
