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
 * Test 5b — close the loop: let the turret chase the goal tag.
 *
 * <p>Do not run this until 4a passed (the geometry is measured and the travel
 * limits hold) and 5a passed (the camera actually sees your tag). The aim loop
 * commands angles, and an angle only means anything once 4a has established
 * what one is worth.
 *
 * <h2>Get the direction right on the first press</h2>
 * Show the tag off to one side. The turret must swing TOWARD it. If it runs the
 * other way it will drive to the travel limit and sit there — press <b>Y</b>
 * immediately to flip INVERT_OUTPUT. This flag is only about how the camera is
 * mounted; the servo's own geometry sign is Constants.Turret.DIRECTION, settled
 * back in 4a.
 *
 * <h2>Then tune out the hunting</h2>
 * kP is a slew rate in degrees per second per degree of tx: at kP = 6, a tag 10
 * degrees off asks for 60 deg/sec. Raise it until the turret closes briskly,
 * then add kD until it stops overshooting. Leave kI at 0 — a turret that briefly
 * loses sight of the tag would wind the integral up while blind and then slam.
 *
 * <p>If it never settles and buzzes back and forth across centre, that is not a
 * kP problem: the smallest correction the loop can make is wider than
 * AIM_TOLERANCE_DEG, so it can never land inside the band. Widen the tolerance.
 *
 * <p>If it tracks smoothly but always lags behind a moving tag, check
 * MAX_SLEW_DEG_PER_S before reaching for kP — the loop may be asking for a rate
 * that is already capped.
 *
 * <h2>Watch the limits</h2>
 * Carry the tag around the robot and watch the angle climb. At MIN/MAX_ANGLE_DEG
 * the turret must stop and hold — it will not follow the tag past its own range,
 * and it should never read LOCKED while pinned there. That is the shot that
 * needs the robot to turn.
 *
 * <p><b>Controls:</b> <b>X</b> blue goal · <b>B</b> red goal · <b>Y</b> flip
 * INVERT_OUTPUT · <b>A</b> park at origin · bumpers nudge kP.
 */
public class TurretAimTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Limelight limelight;
    private Turret turret;
    private boolean parked = false;

    @Override
    public void init() {
        hardware.initTurret(hardwareMap);
        hardware.initLimelight(hardwareMap);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);
        telemetry.addLine("Turret auto-aim. Hand on STOP.");
        telemetry.addLine("If it runs the wrong way, press Y.");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.setState(Turret.State.AUTO_AIM);
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
        }
        if (gamepad1.aWasPressed()) {
            parked = !parked;
        }
        if (gamepad1.rightBumperWasPressed()) {
            TurretTuning.kP += 0.5;
        }
        if (gamepad1.leftBumperWasPressed()) {
            TurretTuning.kP = Math.max(0, TurretTuning.kP - 0.5);
        }

        turret.setState(parked ? Turret.State.RETURN_TO_ORIGIN : Turret.State.AUTO_AIM);

        // Vision must refresh before the turret reads it, or the aim runs a loop
        // behind — which shows up as overshoot you will wrongly blame on kD.
        limelight.periodic();
        turret.periodic();

        telemetry.addData(">> ", parked ? "PARKED (A to resume aiming)"
                : turret.isAtLimit() ? "AT LIMIT — turret can't reach, turn the robot"
                : turret.isOnTarget() ? "LOCKED"
                : limelight.hasTarget() ? "aiming..."
                : "NO TARGET — turret holding still");
        telemetry.addData(">> tx", limelight.hasTarget()
                ? String.format("%+.2f deg  (band +/-%.1f)", limelight.getTx(),
                        TurretTuning.AIM_TOLERANCE_DEG)
                : "--");
        telemetry.addData(">> Angle", "%+.1f deg   (limits %.0f..%.0f)", turret.getAngleDeg(),
                TurretTuning.MIN_ANGLE_DEG, TurretTuning.MAX_ANGLE_DEG);
        telemetry.addLine();
        telemetry.addData("Tag", "%d   (X blue / B red)", limelight.getTargetTagId());
        telemetry.addData("INVERT_OUTPUT", "%s   (Y flips — must turn TOWARD the tag)",
                TurretTuning.INVERT_OUTPUT);
        telemetry.addData("kP", "%.2f deg/s per deg   (bumpers)", TurretTuning.kP);
        telemetry.addData("kD", "%.3f", TurretTuning.kD);
        telemetry.addData("Slew cap", "%.0f deg/s", TurretTuning.MAX_SLEW_DEG_PER_S);
        telemetry.addLine();
        telemetry.addLine("Buzzing at centre and never settling means the");
        telemetry.addLine("smallest correction is wider than AIM_TOLERANCE_DEG.");

        panels.addData("tx", limelight.hasTarget() ? limelight.getTx() : 0.0);
        panels.addData("hasTarget", limelight.hasTarget());
        panels.addData("onTarget", turret.isOnTarget());
        panels.addData("angleDeg", turret.getAngleDeg());
        panels.addData("atLimit", turret.isAtLimit());
        panels.addData("kP", TurretTuning.kP);
        panels.addData("kI", TurretTuning.kI);
        panels.addData("kD", TurretTuning.kD);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        turret.stop();
        hardware.limelight.stop();
    }
}
