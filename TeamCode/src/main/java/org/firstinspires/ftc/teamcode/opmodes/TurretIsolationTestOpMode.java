package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Standalone bring-up test for the turret assembly ONLY — the turret servo, the
 * hood servo, and the Limelight. It uses {@link Hardware#initTurretOnly} so it
 * runs on a Control Hub that has just those three devices in its config; the
 * drivetrain / intake / indexer / flywheel do NOT need to be wired or present.
 *
 * <p>This is the OpMode to run when the turret is handed over on its own, before
 * it's on the full robot. Everything is on gamepad1 (one person, bench test):
 *
 * <ul>
 *   <li><b>Right trigger (hold)</b> — AUTO_AIM: turret tracks the goal tag and
 *       the hood auto-ranges to the measured distance. Needs the Limelight to
 *       see the tag; with no target the turret deliberately holds still.</li>
 *   <li><b>Right stick X</b> — manual turret rotation when not auto-aiming.
 *       START HERE to confirm the servo spins and which way; flip
 *       {@code Constants.Turret.INVERT_OUTPUT} if auto-aim later runs away.</li>
 *   <li><b>Dpad left / right</b> — hood near / far preset.</li>
 *   <li><b>A</b> — stow the hood (default position).</li>
 *   <li><b>X / B</b> — aim at the BLUE (20) / RED (24) goal tag.</li>
 * </ul>
 *
 * <p>SAFETY: the turret is a continuous-rotation servo with no travel limit.
 * Keep clear of the wiring and be ready to hit stop — never leave it in
 * AUTO_AIM with no target while unattended.
 */
@TeleOp(name = "Turret Isolation Test", group = "Test")
public class TurretIsolationTestOpMode extends OpMode {
    // Trigger past this counts as "held".
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Limelight limelight;
    private Turret turret;
    private Hood hood;

    @Override
    public void init() {
        // Minimal init — only the turret, hood, and Limelight are required.
        hardware.initTurretOnly(hardwareMap);

        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);
        hood = new Hood(hardware);

        scheduler.reset();
        // Limelight first so the turret aims on this loop's fresh vision data.
        scheduler.registerSubsystem(limelight, turret, hood);

        telemetry.addLine("Turret isolation test ready.");
        telemetry.addLine("RT = auto-aim, R-stick = manual, dpad L/R = hood presets, A = stow.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Alliance goal selection.
        if (gamepad1.xWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.BLUE_GOAL_TAG);
        }
        if (gamepad1.bWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.RED_GOAL_TAG);
        }

        boolean autoAim = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        if (autoAim) {
            turret.setState(Turret.State.AUTO_AIM);

            double distance = limelight.getDistanceMeters();
            if (distance > 0) {
                hood.setForDistance(distance);
            }
        } else {
            double manual = gamepad1.right_stick_x;
            if (Math.abs(manual) > Constants.Drive.DEADZONE) {
                turret.setState(Turret.State.MANUAL);
                turret.setManualPower(manual);
            } else {
                turret.setState(Turret.State.IDLE);
            }
        }

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

        scheduler.run();

        telemetry.addData("Aiming at", limelight.getTargetTagId() == Constants.Vision.BLUE_GOAL_TAG
                ? "BLUE goal (20)" : "RED goal (24)");
        telemetry.addData("Turret", turret.getState());
        telemetry.addData("Target visible", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
            telemetry.addData("Distance (m)", "%.2f", limelight.getDistanceMeters());
        }
        telemetry.addData("Turret on target", turret.isOnTarget());
        telemetry.addData("Hood position", "%.2f", hood.getCommandedPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        scheduler.cancelAll();
        turret.stop();
        hood.stop();
        hardware.limelight.stop();
    }
}
