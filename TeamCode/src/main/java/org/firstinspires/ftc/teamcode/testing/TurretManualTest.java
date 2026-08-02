package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * Test 4a — positional turret movement and the Axon's programmed limits.
 *
 * <p>The turret is now an Axon in Servo Mode. Its programmer-configured left
 * and right limits protect the wiring, while this test only sends bounded FTC
 * Servo positions. It does not use the old CR-servo encoder/unwrap logic.
 *
 * <p><b>Controls:</b> dpad left/right nudge while held; A sends neutral; B stops
 * commanding a new position; bumpers adjust manual nudge rate. Start with the
 * robot on blocks and watch the first movement in each direction.
 */
public class TurretManualTest extends OpMode {
    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Turret turret;

    @Override
    public void init() {
        hardware.initTurret(hardwareMap);
        turret = new Turret(hardware);
        telemetry.addLine("Positional turret test. Start near the programmed neutral.");
        telemetry.addLine("Dpad nudges; A returns to neutral. Watch for binding.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            turret.setState(Turret.State.MANUAL);
            turret.setManualPower(gamepad1.dpad_left ? -1.0 : 1.0);
        } else if (gamepad1.aWasPressed()) {
            turret.setState(Turret.State.RETURN_TO_ORIGIN);
        } else if (gamepad1.bWasPressed() || turret.getState() == Turret.State.MANUAL) {
            // Releasing dpad leaves the servo holding its last safe position.
            turret.setState(Turret.State.IDLE);
        }

        if (gamepad1.rightBumperWasPressed()) {
            TurretTuning.MANUAL_NUDGE_RATE = Math.min(1.0,
                    TurretTuning.MANUAL_NUDGE_RATE + 0.05);
        }
        if (gamepad1.leftBumperWasPressed()) {
            TurretTuning.MANUAL_NUDGE_RATE = Math.max(0.01,
                    TurretTuning.MANUAL_NUDGE_RATE - 0.05);
        }

        turret.periodic();

        telemetry.addData(">> State", "%s   (dpad nudge / A neutral / B hold)", turret.getState());
        telemetry.addData(">> Commanded input", "%.3f", turret.getCommandedPosition());
        telemetry.addData("Neutral input", "%.3f", TurretTuning.NEUTRAL_POSITION);
        telemetry.addData("Software input band", "%.3f .. %.3f",
                TurretTuning.MIN_POSITION, TurretTuning.MAX_POSITION);
        telemetry.addData("Manual rate", "%.2f position/s   (bumpers)",
                TurretTuning.MANUAL_NUDGE_RATE);
        telemetry.addLine();
        telemetry.addLine("The Axon Programmer's left/right limits are the cable safety.");
        telemetry.addLine("If it binds or reaches a cable limit, stop and tighten those limits.");

        panels.addData("commandedPosition", turret.getCommandedPosition());
        panels.addData("neutralPosition", TurretTuning.NEUTRAL_POSITION);
        panels.addData("manualRate", TurretTuning.MANUAL_NUDGE_RATE);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
