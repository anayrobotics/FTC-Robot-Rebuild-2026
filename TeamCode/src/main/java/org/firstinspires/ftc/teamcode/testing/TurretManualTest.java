package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * Test 4a — the turret, with the camera deliberately left out.
 *
 * <p>The turret is the one subsystem that can destroy itself. It is a
 * continuous-rotation servo with wires running to it and no mechanical
 * knowledge of where it is; everything that stops it twisting its own loom off
 * is software, and all of that software is built on one analog voltage. Prove
 * that voltage before you let vision anywhere near it.
 *
 * <h2>1. Is the feedback wire alive?</h2>
 * Nudge the turret and watch the voltage. It must sweep smoothly and span most
 * of 0 to 3.3 V over a full turn. If it sits at 0, or never moves, the Axon's
 * fourth wire is not connected — and then <b>every safety in this class is
 * inert</b>: the continuous angle never changes, so travel stays 0, so the limit
 * never trips and the turret will happily wind until something tears. Stop and
 * fix the wire.
 *
 * <h2>2. Where is straight ahead?</h2>
 * Push the turret to dead centre by hand and press <b>X</b>. That captures the
 * current raw angle as ORIGIN_DEG, which is the number the park move aims at.
 * Copy it into {@code Constants.Turret.ORIGIN_DEG} afterwards.
 *
 * <h2>3. Does parking go the right way?</h2>
 * Nudge the turret well off centre, then press <b>A</b>. It must drive back
 * toward the origin and stop. If it instead accelerates away and pins itself at
 * the far end, the servo's power-to-angle sign is backwards: press <b>Y</b> to
 * flip INVERT_RETURN and try again. Get this right here — the same sign decides
 * which way the travel limit thinks the turret is winding, so a wrong
 * INVERT_RETURN silently disables the limit in the direction that matters.
 *
 * <h2>4. Does the travel limit hold?</h2>
 * Nudge one way past MAX_TRAVEL_DEG. The nudge must stop working in that
 * direction while still working back the other way. This is the last line of
 * defence for the wiring.
 *
 * <p><b>Controls:</b> <b>dpad left/right</b> nudge · <b>A</b> park at origin ·
 * <b>B</b> stop · <b>X</b> capture ORIGIN_DEG here · <b>Y</b> flip
 * INVERT_RETURN · bumpers change the nudge power.
 *
 * <p><b>Start with the turret near centre.</b> The accumulator assumes it begins
 * inside one half-turn of the origin; it cannot see winding that happened while
 * the robot was switched off.
 */
public class TurretManualTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Turret turret;

    // Span of feedback voltage seen so far — the cheap way to spot a dead wire.
    private double minVolts = Double.MAX_VALUE;
    private double maxVolts = -Double.MAX_VALUE;

    @Override
    public void init() {
        hardware.initTurret(hardwareMap);
        // No Limelight: this test is about the servo and its feedback wire, and
        // a camera fault should not be able to muddy the result.
        turret = new Turret(hardware);
        telemetry.addLine("Turret test (no camera).");
        telemetry.addLine("Start with the turret near centre.");
        telemetry.update();
    }

    /**
     * Runs during INIT too, on purpose. Measuring ORIGIN_DEG means pushing the
     * turret to centre by hand and reading the angle back, and you want to do
     * that with the OpMode not yet started — nothing can command the servo, so
     * it turns freely and cannot surprise you. Without this the readout would
     * be frozen on the init message until you pressed play.
     */
    @Override
    public void init_loop() {
        // State is IDLE, so periodic() reads the encoder and commands 0 power.
        readAndReport();
        turret.periodic();
        telemetry.update();
    }

    @Override
    public void loop() {
        readAndReport();

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            turret.setState(Turret.State.MANUAL);
            turret.setManualPower(gamepad1.dpad_left
                    ? -TurretTuning.MANUAL_NUDGE_POWER
                    : TurretTuning.MANUAL_NUDGE_POWER);
        } else if (gamepad1.aWasPressed()) {
            // Bounce through IDLE so this always counts as a state CHANGE. The
            // stall watch only restarts on a change, so without this a park that
            // gave up could not be retried by pressing A again.
            turret.setState(Turret.State.IDLE);
            turret.setState(Turret.State.RETURN_TO_ORIGIN);
        } else if (gamepad1.bWasPressed() || turret.getState() == Turret.State.MANUAL) {
            // Releasing the dpad drops straight back to IDLE rather than leaving
            // the last nudge power latched on the servo.
            turret.setState(Turret.State.IDLE);
        }

        turret.periodic();
        panels.update(telemetry);
    }

    /** Reads the encoder, handles the settings buttons, and fills telemetry. */
    private void readAndReport() {
        double volts = hardware.turretEncoder.getVoltage();
        minVolts = Math.min(minVolts, volts);
        maxVolts = Math.max(maxVolts, volts);

        if (gamepad1.xWasPressed()) {
            // Turret.updateAngle() notices ORIGIN_DEG changing and re-references
            // the continuous scale, so this takes effect on the next loop.
            TurretTuning.ORIGIN_DEG = turret.getAngleDeg();
        }
        if (gamepad1.yWasPressed()) {
            TurretTuning.INVERT_RETURN = !TurretTuning.INVERT_RETURN;
        }
        if (gamepad1.rightBumperWasPressed()) {
            TurretTuning.MANUAL_NUDGE_POWER = Math.min(1.0, TurretTuning.MANUAL_NUDGE_POWER + 0.05);
        }
        if (gamepad1.leftBumperWasPressed()) {
            TurretTuning.MANUAL_NUDGE_POWER = Math.max(0.05, TurretTuning.MANUAL_NUDGE_POWER - 0.05);
        }

        double travel = turret.getTravelDeg();
        boolean atLimit = Math.abs(travel) >= TurretTuning.MAX_TRAVEL_DEG;
        double span = (maxVolts - minVolts);

        telemetry.addData(">> State", "%s   (dpad nudge / A park / B stop)", turret.getState());
        telemetry.addData(">> Travel", "%+.1f deg of %.0f%s", travel, TurretTuning.MAX_TRAVEL_DEG,
                atLimit ? "   *** AT LIMIT — nudge should refuse this way ***" : "");
        telemetry.addData(">> At origin", turret.isAtOrigin());
        if (turret.isStalled()) {
            telemetry.addLine("!! GAVE UP — the turret stopped moving under power.");
            telemetry.addLine("!! Something is blocking it, or the power is too low");
            telemetry.addLine("!! for the load. Press A again to retry.");
        }
        telemetry.addLine();
        telemetry.addData("Feedback", "%.3f V of %.2f V max  ->  raw %.1f deg",
                volts, hardware.turretEncoder.getMaxVoltage(), turret.getAngleDeg());
        telemetry.addData("Voltage seen", "%.3f .. %.3f  (span %.3f V)", minVolts, maxVolts, span);
        if (span < 0.05) {
            telemetry.addLine("!! FEEDBACK WIRE LOOKS DEAD — voltage is not moving.");
            telemetry.addLine("!! The travel limit and park CANNOT work. Fix before");
            telemetry.addLine("!! running auto-aim, or the turret will wind up.");
        }
        telemetry.addLine();
        telemetry.addData("ORIGIN_DEG", "%.1f   (X captures here)", TurretTuning.ORIGIN_DEG);
        telemetry.addData("INVERT_RETURN", "%s   (Y flips — park must go TOWARD origin)",
                TurretTuning.INVERT_RETURN);
        telemetry.addData("Nudge power", "%.2f   (bumpers)", TurretTuning.MANUAL_NUDGE_POWER);
        telemetry.addLine();
        telemetry.addLine("Copy ORIGIN_DEG and INVERT_RETURN into Constants.Turret.");

        panels.addData("volts", volts);
        panels.addData("rawAngleDeg", turret.getAngleDeg());
        panels.addData("travelDeg", travel);
        panels.addData("atOrigin", turret.isAtOrigin());
        panels.addData("ORIGIN_DEG", TurretTuning.ORIGIN_DEG);
        panels.addData("MAX_TRAVEL_DEG", TurretTuning.MAX_TRAVEL_DEG);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
