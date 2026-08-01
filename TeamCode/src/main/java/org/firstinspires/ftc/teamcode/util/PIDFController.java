package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

// Velocity PIDF controller: output = kF*target + kP*error + kI*integral + kD*derivative.
// Uses wall-clock dt so it is loop-time independent.
public class PIDFController {
    private double kP, kI, kD, kF;

    private double integral = 0;
    private double lastError = 0;
    private boolean firstRun = true;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        setCoefficients(kP, kI, kD, kF);
    }

    public void setCoefficients(double kP, double kI, double kD, double kF) {
        // Turning the integral on or off starts it from scratch. Every loop in
        // this robot ships with kI = 0, and the tuning OpModes invite you to
        // raise it live — if the accumulator had been quietly running unused all
        // along, typing a gain in would apply thirty seconds of stored error in
        // a single frame and slam whatever it drives.
        if (kI != this.kI) {
            integral = 0;
        }
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculate(double target, double measured) {
        double dt = firstRun ? 0 : timer.seconds();
        timer.reset();
        firstRun = false;

        double error = target - measured;
        double derivative = 0;
        if (dt > 0) {
            // Only accumulate while the term is actually in use — see
            // setCoefficients().
            if (kI != 0) {
                integral += error * dt;
            }
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        return kF * target + kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        firstRun = true;
        timer.reset();
    }
}
