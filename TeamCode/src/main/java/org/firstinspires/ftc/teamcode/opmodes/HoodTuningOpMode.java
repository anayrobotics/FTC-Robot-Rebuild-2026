package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.tuning.HoodTuning;

/**
 * Hood position tuning OpMode, wired to Panels.
 *
 * <p>Edit {@code HoodTuning.TEST_POSITION} (and the bounds / presets) live in
 * the Panels "Configurables" tab while this runs — the {@link Hood} clamps to
 * the live bounds every loop.
 *
 * <p>Controls: <b>A</b> drive to TEST_POSITION, <b>X</b> near preset,
 * <b>B</b> far preset, <b>Y</b> stow (default). Nudge TEST_POSITION on Panels
 * to find each shot's angle, then copy the numbers into {@link
 * org.firstinspires.ftc.teamcode.Constants.Hood}.
 */
@TeleOp(name = "Hood Position Tuning", group = "Tuning")
public class HoodTuningOpMode extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Hood hood;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        hood = new Hood(hardware);
        panels.debug("Hood tuning ready. A = TEST_POSITION, X = near, B = far, Y = stow.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            hood.setPosition(HoodTuning.TEST_POSITION);
        }
        if (gamepad1.x) {
            hood.setNearPreset();
        }
        if (gamepad1.b) {
            hood.setFarPreset();
        }
        if (gamepad1.y) {
            hood.stop();
        }

        // Writes the (clamped) position to the servo with the live bounds.
        hood.periodic();

        panels.addData("target", hood.getTargetPosition());
        panels.addData("commanded", hood.getCommandedPosition());
        panels.addData("TEST_POSITION", HoodTuning.TEST_POSITION);
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
