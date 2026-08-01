package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * Turret aim-PID tuning OpMode, wired to Panels.
 *
 * <p>Edit {@code TurretTuning.kP/kI/kD/kF} live in the Panels "Configurables"
 * tab while this runs — the {@link Turret} reads them every loop. The turret is
 * held in AUTO_AIM, so point a goal AprilTag at the Limelight and watch {@code tx}
 * drive toward 0 in the Panels graph view. Tune kP first, then kD to damp the
 * oscillation; leave kI at 0.
 *
 * <p>Requires a visible goal tag to move — with no target it deliberately stops.
 *
 * <h2>Measuring the turret origin</h2>
 * This opmode also reports {@code rawAngleDeg} straight off the servo's
 * feedback wire, which is how you set the park position up:
 * <ol>
 *   <li>Init (don't start — the turret stays put with the aim loop idle).</li>
 *   <li>Push the turret to dead centre by hand.</li>
 *   <li>Read {@code rawAngleDeg} and copy it into {@code TurretTuning.ORIGIN_DEG}
 *       on Panels, then into {@link org.firstinspires.ftc.teamcode.Constants.Turret}
 *       once it parks where you want.</li>
 * </ol>
 */
@TeleOp(name = "Turret PID Tuning", group = "Tuning")
public class TurretTuningOpMode extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Limelight limelight;
    private Turret turret;

    @Override
    public void init() {
        hardware.init(hardwareMap);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);
        panels.debug("Turret PID tuning ready. Show a goal tag to the Limelight.");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        turret.setState(Turret.State.AUTO_AIM);
    }

    @Override
    public void loop() {
        // Limelight must refresh before the turret reads it this loop.
        limelight.periodic();
        turret.periodic();

        panels.addData("tx", limelight.hasTarget() ? limelight.getTx() : 0.0);
        panels.addData("hasTarget", limelight.hasTarget());
        panels.addData("onTarget", turret.isOnTarget());
        // Raw feedback angle — this is the number to read off when measuring
        // ORIGIN_DEG. See the class docs.
        panels.addData("rawAngleDeg", turret.getAngleDeg());
        panels.addData("originErrorDeg", turret.getOriginErrorDeg());
        panels.addData("kP", TurretTuning.kP);
        panels.addData("kI", TurretTuning.kI);
        panels.addData("kD", TurretTuning.kD);
        panels.addData("kF", TurretTuning.kF);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
