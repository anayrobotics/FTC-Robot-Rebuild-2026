package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.tuning.HoodTuning;

/**
 * Test 3b — find the hood's real travel limits, then its shot angles.
 *
 * <h2>Find the limits first, carefully</h2>
 * {@code MIN_POSITION} and {@code MAX_POSITION} default to 0.15 and 0.85, which
 * are guesses. A positional servo asked to go past its linkage's mechanical stop
 * does not give up — it holds full torque against the stop until something
 * strips, and servos are the part most likely to die quietly on a test day.
 *
 * <p>So: start at the stowed default and nudge outward <b>one 0.01 step at a
 * time</b>, hand on the stop switch, listening. The moment the servo starts to
 * buzz or the linkage stops moving, you have gone one step too far — back off
 * two and record that as the limit. Do both ends. Then tighten MIN/MAX to those
 * numbers, because from that point on the clamp protects you from every bad
 * preset and every stale auto-range value for the rest of the season.
 *
 * <h2>Then set the shot angles</h2>
 * The near and far presets, and the distance table in {@code Constants.Hood},
 * only mean something once you are shooting at a real goal — come back to those
 * after test 6a.
 *
 * <p><b>Controls:</b> bumpers move the hood +/- 0.01 · dpad up/down +/- 0.05 ·
 * <b>X</b> near preset · <b>B</b> far preset · <b>Y</b> stow · <b>A</b> stores
 * the current position as MIN, <b>dpad right</b> stores it as MAX.
 */
public class HoodTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Hood hood;

    @Override
    public void init() {
        hardware.initHood(hardwareMap);
        hood = new Hood(hardware);
        // The hood is where Hardware.initHood left it: the stowed default. Start
        // the test from there rather than jumping somewhere new.
        HoodTuning.TEST_POSITION = Constants.Hood.DEFAULT_POSITION;
        hood.setPosition(HoodTuning.TEST_POSITION);
        telemetry.addLine("Hood test. Nudge ONE STEP at a time toward the stops.");
        telemetry.addLine("Hand on the stop switch. A buzzing servo is stalling.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double nudge = 0;
        if (gamepad1.rightBumperWasPressed()) {
            nudge = 0.01;
        }
        if (gamepad1.leftBumperWasPressed()) {
            nudge = -0.01;
        }
        if (gamepad1.dpadUpWasPressed()) {
            nudge = 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            nudge = -0.05;
        }
        if (nudge != 0) {
            HoodTuning.TEST_POSITION = Range.clip(HoodTuning.TEST_POSITION + nudge, 0, 1);
            hood.setPosition(HoodTuning.TEST_POSITION);
        }

        if (gamepad1.xWasPressed()) {
            hood.setNearPreset();
        }
        if (gamepad1.bWasPressed()) {
            hood.setFarPreset();
        }
        if (gamepad1.yWasPressed()) {
            hood.stop();
        }

        // Record the safe band. Widening the clamp has to be allowed or you
        // could never explore outward past the current guess, so the clamp is
        // only re-applied to whatever the hood is holding.
        if (gamepad1.aWasPressed()) {
            HoodTuning.MIN_POSITION = hood.getTargetPosition();
        }
        if (gamepad1.dpadRightWasPressed()) {
            HoodTuning.MAX_POSITION = hood.getTargetPosition();
        }

        hood.periodic();

        boolean clamped = Math.abs(hood.getTargetPosition() - hood.getCommandedPosition()) > 1e-6;

        telemetry.addData(">> Commanded", "%.3f%s", hood.getCommandedPosition(),
                clamped ? "  (CLAMPED — asked for " + String.format("%.3f", hood.getTargetPosition()) + ")" : "");
        telemetry.addData(">> Safe band", "%.3f .. %.3f   (A = set MIN, dpad right = set MAX)",
                HoodTuning.MIN_POSITION, HoodTuning.MAX_POSITION);
        telemetry.addLine();
        telemetry.addData("Presets", "near %.3f (X)   far %.3f (B)   stow %.3f (Y)",
                HoodTuning.NEAR_PRESET, HoodTuning.FAR_PRESET, Constants.Hood.DEFAULT_POSITION);
        telemetry.addLine();
        telemetry.addLine("Bumpers +/-0.01, dpad up/down +/-0.05.");
        telemetry.addLine("Back off TWO steps from wherever it buzzes,");
        telemetry.addLine("then copy MIN/MAX into Constants.Hood.");

        panels.addData("target", hood.getTargetPosition());
        panels.addData("commanded", hood.getCommandedPosition());
        panels.addData("MIN_POSITION", HoodTuning.MIN_POSITION);
        panels.addData("MAX_POSITION", HoodTuning.MAX_POSITION);
        panels.addData("NEAR_PRESET", HoodTuning.NEAR_PRESET);
        panels.addData("FAR_PRESET", HoodTuning.FAR_PRESET);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        hood.stop();
        hood.periodic();
    }
}
