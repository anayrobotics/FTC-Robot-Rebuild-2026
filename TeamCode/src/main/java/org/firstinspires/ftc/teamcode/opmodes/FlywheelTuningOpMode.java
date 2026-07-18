package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuning;

/**
 * Flywheel velocity-PIDF tuning OpMode, wired to Panels.
 *
 * <p>Edit {@code FlywheelTuning.kP/kI/kD/kF} (and {@code TEST_RPM}) live in the
 * Panels "Configurables" tab while this runs — the {@link Flywheel} reads them
 * every loop. Watch {@code targetRpm} vs {@code actualRpm} in the Panels graph
 * view to dial the gains in. Tune kF first (open-loop speed), then kP, then kD.
 *
 * <p>Controls: <b>A</b> spin up to TEST_RPM, <b>B</b> stop.
 */
@TeleOp(name = "Flywheel PID Tuning", group = "Tuning")
public class FlywheelTuningOpMode extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Flywheel flywheel;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        flywheel = new Flywheel(hardware);
        panels.debug("Flywheel PID tuning ready. A = spin up, B = stop.");
        panels.update(telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            flywheel.setTargetRpm(FlywheelTuning.TEST_RPM);
        }
        if (gamepad1.b) {
            flywheel.stop();
        }

        // Runs the velocity PIDF with the live gains from FlywheelTuning.
        flywheel.periodic();

        // Numeric telemetry keys are plotted over time in the Panels graph view.
        panels.addData("targetRpm", flywheel.getTargetRpm());
        panels.addData("actualRpm", flywheel.getCurrentRpm());
        panels.addData("atTarget", flywheel.atTargetRpm());
        panels.addData("kP", FlywheelTuning.kP);
        panels.addData("kI", FlywheelTuning.kI);
        panels.addData("kD", FlywheelTuning.kD);
        panels.addData("kF", FlywheelTuning.kF);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        flywheel.stop();
        flywheel.periodic();
    }
}
