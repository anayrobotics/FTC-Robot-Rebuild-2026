package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * <p>The flywheel runs OPEN-LOOP with each motor on its OWN trigger, so you can
 * spin one at a time and see which way it turns — that's how you find out which
 * motor needs {@code Constants.Flywheel.*_DIRECTION} reversed so both drive the
 * wheel together. There is deliberately no "spin both" control. As a safety, if
 * BOTH triggers are pressed at once neither motor runs.
 *
 * <p>Everything is on gamepad1 (one person, bench test):
 * <ul>
 *   <li><b>Left trigger</b> — spin the LEFT flywheel motor (power = trigger).</li>
 *   <li><b>Right trigger</b> — spin the RIGHT flywheel motor (power = trigger).</li>
 *   <li><b>Dpad left / right</b> — hood near / far preset.</li>
 *   <li><b>A</b> — stow the hood (default position).</li>
 * </ul>
 *
 * <p>SAFETY: the flywheel spins fast. Keep clear of it and be ready to hit stop.
 */
@TeleOp(name = "Flywheel + Hood Test", group = "Test")
public class FlywheelHoodTestOpMode extends OpMode {
    // Trigger past this counts as "pressed" for the both-at-once safety check.
    private static final double TRIGGER_THRESHOLD = 0.1;

    private final Hardware hardware = new Hardware();

    private Flywheel flywheel;
    private Hood hood;

    @Override
    public void init() {
        // Minimal init — only the two flywheel motors and the hood servo.
        hardware.initFlywheelAndHood(hardwareMap);

        flywheel = new Flywheel(hardware);
        hood = new Hood(hardware);

        telemetry.addLine("Flywheel + hood test ready.");
        telemetry.addLine("Left trigger = LEFT motor, Right trigger = RIGHT motor.");
        telemetry.addLine("Dpad L/R = hood near/far, A = stow.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // One motor per trigger so you can spin each on its own. Guard against
        // running both at once — if both triggers are down, stop both.
        boolean bothPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD
                && gamepad1.right_trigger > TRIGGER_THRESHOLD;
        double leftCmd = bothPressed ? 0 : gamepad1.left_trigger;
        double rightCmd = bothPressed ? 0 : gamepad1.right_trigger;
        flywheel.setMotorPowers(leftCmd, rightCmd);

        // Hood presets / stow.
        if (gamepad1.dpadLeftWasPressed()) {
            hood.setNearPreset();
        }
        if (gamepad1.dpadRightWasPressed()) {
            hood.setFarPreset();
        }
        if (gamepad1.aWasPressed()) {
            hood.stop();
        }

        // Write the commanded powers / position to the hardware this loop.
        flywheel.periodic();
        hood.periodic();

        if (bothPressed) {
            telemetry.addLine(">> BOTH triggers pressed — motors held OFF. Use one at a time.");
        }
        telemetry.addData("Left motor power", "%.2f", flywheel.getLeftPower());
        telemetry.addData("Right motor power", "%.2f", flywheel.getRightPower());
        telemetry.addData("RPM (left encoder)", "%.0f", flywheel.getCurrentRpm());
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
