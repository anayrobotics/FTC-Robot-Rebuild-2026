package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * Test 5b — positional-servo auto aim. The Axon's internal left/right limits
 * must already be programmed and verified in TurretManualTest before this runs.
 *
 * <p>The test starts at neutral and never tracks until A is pressed. This makes
 * the first on-robot direction check deliberate instead of moving as soon as
 * START is pressed.
 *
 * <p><b>Controls:</b> X blue goal; B red goal; A enables/disables tracking;
 * Y flips the aim direction and disables tracking; bumpers tune kP. If the
 * selected tag leaves the camera image, AUTO_AIM commands the servo's neutral
 * input.
 */
public class TurretAimTest extends OpMode {
    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Limelight limelight;
    private Turret turret;
    // Begin in a known safe state. The operator must deliberately enable aiming.
    private boolean aimingEnabled = false;

    @Override
    public void init() {
        hardware.initTurret(hardwareMap);
        hardware.initLimelight(hardwareMap);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);
        telemetry.addLine("Starts at neutral. A enables tracking; hand on STOP.");
        telemetry.addLine("If it moves away from the tag: Y, then A to retest.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Command neutral as soon as the OpMode starts, before the operator
        // opts in to camera tracking.
        turret.setState(Turret.State.RETURN_TO_ORIGIN);
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.BLUE_GOAL_TAG);
        }
        if (gamepad1.bWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.RED_GOAL_TAG);
        }
        if (gamepad1.yWasPressed()) {
            TurretTuning.INVERT_OUTPUT = !TurretTuning.INVERT_OUTPUT;
            // Changing sign while tracking would make an immediate uncontrolled
            // reverse move. Return neutral first; A explicitly begins the retest.
            aimingEnabled = false;
        }
        if (gamepad1.aWasPressed()) {
            aimingEnabled = !aimingEnabled;
        }
        if (gamepad1.rightBumperWasPressed()) {
            TurretTuning.kP += 0.002;
        }
        if (gamepad1.leftBumperWasPressed()) {
            TurretTuning.kP = Math.max(0, TurretTuning.kP - 0.002);
        }

        turret.setState(aimingEnabled ? Turret.State.AUTO_AIM : Turret.State.RETURN_TO_ORIGIN);
        limelight.periodic();
        turret.periodic();

        telemetry.addData(">> ", !aimingEnabled ? "NEUTRAL — press A to enable aiming"
                : turret.isOnTarget() ? "LOCKED"
                : limelight.hasTarget() ? "aiming..."
                : "NO TARGET — RETURNING NEUTRAL");
        telemetry.addData(">> tx", limelight.hasTarget()
                ? String.format("%+.2f deg  (band +/-%.1f)", limelight.getTx(),
                        TurretTuning.AIM_TOLERANCE_DEG)
                : "--");
        telemetry.addData("Commanded input", "%.3f   neutral %.3f", turret.getCommandedPosition(),
                TurretTuning.NEUTRAL_POSITION);
        telemetry.addData("Tag", "%d   (X blue / B red)", limelight.getTargetTagId());
        telemetry.addData("Aiming enabled", "%s   (A toggles / Y disables)", aimingEnabled);
        telemetry.addData("INVERT_OUTPUT", "%s   (Y flips + returns neutral)",
                TurretTuning.INVERT_OUTPUT);
        telemetry.addData("kP", "%.4f position/s/deg   (bumpers)", TurretTuning.kP);
        telemetry.addData("Max aim rate", "%.3f position/s", TurretTuning.MAX_AIM_RATE);
        telemetry.addLine("After a successful direction test, copy the final inversion into Constants.");

        panels.addData("tx", limelight.hasTarget() ? limelight.getTx() : 0.0);
        panels.addData("hasTarget", limelight.hasTarget());
        panels.addData("onTarget", turret.isOnTarget());
        panels.addData("aimingEnabled", aimingEnabled);
        panels.addData("commandedPosition", turret.getCommandedPosition());
        panels.addData("neutralPosition", TurretTuning.NEUTRAL_POSITION);
        panels.addData("kP", TurretTuning.kP);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        turret.stop();
        hardware.limelight.stop();
    }
}
