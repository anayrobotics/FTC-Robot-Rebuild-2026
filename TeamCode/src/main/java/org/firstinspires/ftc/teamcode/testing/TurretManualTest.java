package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * Test 4a — the turret geometry, with the camera deliberately left out.
 *
 * <p>The turret runs on a positional servo with no feedback of any kind, so the
 * code's idea of where it is pointing is only ever as good as two numbers:
 * ORIGIN_POSITION (which servo position is straight ahead) and SERVO_RANGE_DEG
 * (how many degrees of turret the servo's full travel is worth). Measure both
 * here, before vision is anywhere near it. Get SERVO_RANGE_DEG wrong and every
 * angle is wrong by the same factor — the turret will consistently under- or
 * over-shoot and no amount of gain tuning in 5b will fix it.
 *
 * <h2>1. Where is straight ahead?</h2>
 * <b>Dpad left/right</b> jogs the turret. Walk it until it points dead down the
 * robot's centreline, then press <b>X</b>. That re-references the origin to
 * here, so the angle readout zeroes. Copy the reported ORIGIN_POSITION into
 * {@code Constants.Turret.ORIGIN_POSITION}.
 *
 * <h2>2. Is SERVO_RANGE_DEG right?</h2>
 * From the origin, press <b>A</b> to command exactly +90 degrees. Measure what
 * the turret ACTUALLY swung with a protractor. If it moved 60 when it was asked
 * for 90, SERVO_RANGE_DEG is too big by that ratio — scale it by 60/90 and
 * repeat until the commanded and measured angles agree. Bumpers nudge it live.
 *
 * <h2>3. Do the travel limits hold?</h2>
 * Jog one way and keep going. The turret must stop at MAX_ANGLE_DEG and refuse
 * to go further that way while still jogging back the other. Then check it
 * against the real mechanical range: the servo must reach both software limits
 * without ever touching a hard stop. If it strains, TIGHTEN the limits — they
 * are the only thing protecting the linkage.
 *
 * <h2>4. Is MAX_SLEW_DEG_PER_S honest?</h2>
 * Press <b>B</b> to park from a long way out. The turret must arrive at centre
 * at the same moment the angle readout reaches 0. If the readout gets there
 * first and the turret is still visibly swinging, the slew cap is faster than
 * the servo really is — lower it. Everything downstream assumes the commanded
 * angle is where the turret is.
 *
 * <p><b>Controls:</b> <b>dpad left/right</b> jog · <b>X</b> set origin here ·
 * <b>A</b> command +90 · <b>B</b> park at origin · <b>Y</b> stop ·
 * bumpers change SERVO_RANGE_DEG.
 *
 * <p><b>Start with the turret pointed straight ahead.</b> Nothing surveys it:
 * the code assumes it begins at the origin, so if it doesn't, everything on
 * screen is offset by however far out it was.
 */
public class TurretManualTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Turret turret;

    // Set by A: hold the turret at +90 so it can be measured against a
    // protractor. Cleared by any jog or park.
    private boolean checkingRange = false;

    @Override
    public void init() {
        hardware.initTurret(hardwareMap);
        // No Limelight: this test is about the servo and its geometry, and a
        // camera fault should not be able to muddy the result.
        turret = new Turret(hardware);
        telemetry.addLine("Turret geometry test (no camera).");
        telemetry.addLine("Start with the turret pointed straight ahead.");
        telemetry.addLine("Nothing is commanded until you press PLAY.");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleButtons();

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            checkingRange = false;
            turret.setState(Turret.State.MANUAL);
            turret.setManualRate(gamepad1.dpad_left
                    ? -TurretTuning.MANUAL_NUDGE_DEG_PER_S
                    : TurretTuning.MANUAL_NUDGE_DEG_PER_S);
        } else if (checkingRange) {
            // Jog toward +90 and stop there. Reusing MANUAL rather than adding a
            // state to the subsystem: the point is to park at a known angle and
            // let a protractor disagree with it.
            double error = 90.0 - turret.getAngleDeg();
            turret.setState(Turret.State.MANUAL);
            turret.setManualRate(Math.abs(error) < 0.5 ? 0
                    : Math.copySign(TurretTuning.MANUAL_NUDGE_DEG_PER_S, error));
        } else if (turret.getState() == Turret.State.MANUAL) {
            // Releasing the dpad drops straight back to IDLE rather than leaving
            // the last jog rate latched on.
            turret.setState(Turret.State.IDLE);
        }

        turret.periodic();
        report();
        panels.update(telemetry);
    }

    private void handleButtons() {
        if (gamepad1.xWasPressed()) {
            // Re-reference the origin to wherever the turret is now. The
            // subsystem converts angles through ORIGIN_POSITION every loop, so
            // moving it here shifts the whole scale and the angle reads 0.
            TurretTuning.ORIGIN_POSITION = turret.getPosition();
        }
        if (gamepad1.aWasPressed()) {
            checkingRange = true;
        }
        if (gamepad1.bWasPressed()) {
            checkingRange = false;
            turret.setState(Turret.State.RETURN_TO_ORIGIN);
        }
        if (gamepad1.yWasPressed()) {
            checkingRange = false;
            turret.setState(Turret.State.IDLE);
        }
        if (gamepad1.rightBumperWasPressed()) {
            TurretTuning.SERVO_RANGE_DEG += 5.0;
        }
        if (gamepad1.leftBumperWasPressed()) {
            TurretTuning.SERVO_RANGE_DEG = Math.max(5.0, TurretTuning.SERVO_RANGE_DEG - 5.0);
        }
    }

    private void report() {
        double angle = turret.getAngleDeg();

        telemetry.addData(">> State", "%s   (dpad jog / A +90 / B park / Y stop)",
                turret.getState());
        telemetry.addData(">> Angle", "%+.1f deg   (limits %.0f..%.0f)%s",
                angle, TurretTuning.MIN_ANGLE_DEG, TurretTuning.MAX_ANGLE_DEG,
                turret.isAtLimit() ? "   *** AT LIMIT — jog should refuse this way ***" : "");
        telemetry.addData(">> At origin", turret.isAtOrigin());
        if (checkingRange) {
            telemetry.addLine(">> Holding +90 — measure the REAL swing with a");
            telemetry.addLine(">> protractor. Short of 90 means SERVO_RANGE_DEG");
            telemetry.addLine(">> is too big; scale it by measured/90.");
        }
        telemetry.addLine();
        telemetry.addData("Servo position", "%.4f", turret.getPosition());
        telemetry.addData("ORIGIN_POSITION", "%.4f   (X sets it to here)",
                TurretTuning.ORIGIN_POSITION);
        telemetry.addData("SERVO_RANGE_DEG", "%.0f   (bumpers)", TurretTuning.SERVO_RANGE_DEG);
        telemetry.addLine();
        telemetry.addLine("Copy ORIGIN_POSITION and SERVO_RANGE_DEG into");
        telemetry.addLine("Constants.Turret, then tighten MIN/MAX_ANGLE_DEG");
        telemetry.addLine("to the real mechanical range.");

        panels.addData("angleDeg", angle);
        panels.addData("position", turret.getPosition());
        panels.addData("atOrigin", turret.isAtOrigin());
        panels.addData("atLimit", turret.isAtLimit());
        panels.addData("ORIGIN_POSITION", TurretTuning.ORIGIN_POSITION);
        panels.addData("SERVO_RANGE_DEG", TurretTuning.SERVO_RANGE_DEG);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
