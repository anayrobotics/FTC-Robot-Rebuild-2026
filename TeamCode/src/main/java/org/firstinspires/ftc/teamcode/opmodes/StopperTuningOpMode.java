package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.tuning.StopperTuning;

/**
 * Stopper gate tuning OpMode, wired to Panels.
 *
 * <p>Edit {@code StopperTuning.BLOCKING_POSITION} / {@code OPEN_POSITION} live in
 * the Panels "Configurables" tab while this runs — the {@link Stopper} re-reads
 * them every loop, so the gate moves as you nudge the value it's currently
 * sitting at.
 *
 * <p>Controls: <b>A</b> open, <b>B</b> block. Find the position that just barely
 * holds a ball back and the one that fully clears the path, then copy both into
 * {@link org.firstinspires.ftc.teamcode.Constants.Stopper}.
 */
@TeleOp(name = "Stopper Position Tuning", group = "Tuning")
public class StopperTuningOpMode extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Stopper stopper;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        stopper = new Stopper(hardware);
        panels.debug("Stopper tuning ready. A = open, B = block.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            stopper.open();
        }
        if (gamepad1.b) {
            stopper.block();
        }

        // Writes the live position for the current state to the servo.
        stopper.periodic();

        panels.addData("state", stopper.getState());
        panels.addData("commanded", stopper.getCommandedPosition());
        panels.addData("settled", stopper.isSettled());
        panels.addData("BLOCKING_POSITION", StopperTuning.BLOCKING_POSITION);
        panels.addData("OPEN_POSITION", StopperTuning.OPEN_POSITION);
        panels.addData("TRAVEL_TIME_S", StopperTuning.TRAVEL_TIME_S);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        stopper.stop();
        stopper.periodic();
    }
}
