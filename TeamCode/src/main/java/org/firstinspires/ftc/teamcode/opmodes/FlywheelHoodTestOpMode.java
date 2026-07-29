package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

/**
 * Standalone bring-up test for the shooter assembly ONLY — the two flywheel
 * motors and the hood servo. It uses {@link Hardware#initFlywheelAndHood} so it
 * runs on a Control Hub that has just those three devices in its config; the
 * drivetrain / intake / indexer / turret / Limelight / IMU do NOT need to be
 * wired or present (the full init would throw on the first missing device).
 *
 * <p>The flywheel runs OPEN-LOOP: you set a raw motor power and both motors spin
 * together — there is no velocity PID to tune. The measured RPM is shown on
 * telemetry so you can nudge the power up until the wheel is at the speed you
 * want.
 *
 * <p>Everything is on gamepad1 (one person, bench test):
 * <ul>
 *   <li><b>Right bumper</b> — flywheel power +5%.</li>
 *   <li><b>Left bumper</b> — flywheel power −5%.</li>
 *   <li><b>B</b> — flywheel stop (power 0, coasts down).</li>
 *   <li><b>Dpad up / down</b> — hood position +/− 0.05 (clamped to its travel band).</li>
 * </ul>
 *
 * <p>SAFETY: the flywheel spins fast. Keep clear of it and be ready to hit stop.
 */
@TeleOp(name = "Flywheel + Hood Test", group = "Test")
public class FlywheelHoodTestOpMode extends OpMode {
    // How much each bumper press changes the open-loop flywheel power.
    private static final double POWER_STEP = 0.05;
    // How much each dpad press moves the hood.
    private static final double HOOD_STEP = 0.05;

    private final Hardware hardware = new Hardware();

    private Flywheel flywheel;
    private Hood hood;

    // Held open-loop power setpoint, adjusted with the bumpers.
    private double flywheelPower = 0;
    // Held hood position setpoint, adjusted with the dpad.
    private double hoodPosition = Constants.Hood.DEFAULT_POSITION;

    @Override
    public void init() {
        // Minimal init — only the two flywheel motors and the hood servo.
        hardware.initFlywheelAndHood(hardwareMap);

        flywheel = new Flywheel(hardware);
        hood = new Hood(hardware);

        telemetry.addLine("Flywheel + hood test ready.");
        telemetry.addLine("Bumpers = flywheel power +/-, B = stop.");
        telemetry.addLine("Dpad up/down = hood +/- 0.05.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Flywheel open-loop power (bumpers nudge, B stops). Both motors spin.
        if (gamepad1.rightBumperWasPressed()) {
            flywheelPower = Range.clip(flywheelPower + POWER_STEP, 0, 1);
        }
        if (gamepad1.leftBumperWasPressed()) {
            flywheelPower = Range.clip(flywheelPower - POWER_STEP, 0, 1);
        }
        if (gamepad1.bWasPressed()) {
            flywheelPower = 0;
        }
        flywheel.setOpenLoopPower(flywheelPower);

        // Hood position nudge (dpad up/down), clamped to the travel band.
        if (gamepad1.dpadUpWasPressed()) {
            hoodPosition = Range.clip(hoodPosition + HOOD_STEP,
                    Constants.Hood.MIN_POSITION, Constants.Hood.MAX_POSITION);
        }
        if (gamepad1.dpadDownWasPressed()) {
            hoodPosition = Range.clip(hoodPosition - HOOD_STEP,
                    Constants.Hood.MIN_POSITION, Constants.Hood.MAX_POSITION);
        }
        hood.setPosition(hoodPosition);

        // Write the commanded power / position to the hardware this loop.
        flywheel.periodic();
        hood.periodic();

        telemetry.addData("Flywheel power", "%.2f", flywheel.getOpenLoopPower());
        telemetry.addData("Flywheel RPM", "%.0f", flywheel.getCurrentRpm());
        telemetry.addData("Hood position", "%.2f", hood.getCommandedPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
        flywheel.periodic();
        hood.stop();
        hood.periodic();
    }
}
