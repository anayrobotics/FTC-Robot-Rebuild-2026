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
 * <p>Do not run this until 4a passed (the feedback wire is alive and the travel
 * limit holds) and 5a passed (the camera actually sees your tag). If either is
 * unproven, an auto-aiming CRServo is a machine for winding your own wiring into
 * a knot.
 *
 * <h2>Get the direction right on the first press</h2>
 * Show the tag off to one side. The turret must swing TOWARD it. If it runs the
 * other way it will accelerate to the travel limit — press <b>Y</b> immediately
 * to flip INVERT_OUTPUT. Note this is a different flag from the one in test 4a:
 * that one was about the servo's gearing, this one is about how the camera is
 * mounted, and getting one right tells you nothing about the other.
 *
 * <h2>Then tune out the hunting</h2>
 * Raise kP until it reaches the target briskly, then add kD until it stops
 * overshooting. Leave kI at 0: a turret that briefly loses sight of the tag
 * would wind the integral up while blind and then slam.
 *
 * <p>If it never settles and buzzes back and forth across centre, that is not
 * a kP problem — it is MIN_AIM_POWER kicking the turret further than
 * AIM_TOLERANCE_DEG is wide, so it can never land inside the band. Widen the
 * tolerance or drop the floor.
 *
 * <h2>Watch the wind</h2>
 * Carry the tag in a circle around the robot and watch the travel figure climb.
 * At the limit the turret should UNWRAP — swing a full turn the other way to the
 * same heading — and come out still pointed at the tag. Confirm it does that,
 * and confirm it never reads LOCKED mid-swing.
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
            TurretTuning.kP += 0.002;
        }
        if (gamepad1.leftBumperWasPressed()) {
            TurretTuning.kP = Math.max(0, TurretTuning.kP - 0.002);
        }

        turret.setState(parked ? Turret.State.RETURN_TO_ORIGIN : Turret.State.AUTO_AIM);

        // Vision must refresh before the turret reads it, or the aim runs a loop
        // behind — which shows up as overshoot you will wrongly blame on kD.
        limelight.periodic();
        turret.periodic();

        telemetry.addData(">> ", parked ? "PARKED (A to resume aiming)"
                : turret.isUnwinding() ? "UNWRAPPING — full turn the other way"
                : turret.isOnTarget() ? "LOCKED"
                : limelight.hasTarget() ? "aiming..."
                : "NO TARGET — turret holding still");
        telemetry.addData(">> tx", limelight.hasTarget()
                ? String.format("%+.2f deg  (band +/-%.1f)", limelight.getTx(),
                        TurretTuning.AIM_TOLERANCE_DEG)
                : "--");
        telemetry.addData(">> Travel", "%+.1f deg of %.0f", turret.getTravelDeg(),
                TurretTuning.MAX_TRAVEL_DEG);
        telemetry.addLine();
        telemetry.addData("Tag", "%d   (X blue / B red)", limelight.getTargetTagId());
        telemetry.addData("INVERT_OUTPUT", "%s   (Y flips — must turn TOWARD the tag)",
                TurretTuning.INVERT_OUTPUT);
        telemetry.addData("kP", "%.4f   (bumpers)", TurretTuning.kP);
        telemetry.addData("kD", "%.5f", TurretTuning.kD);
        telemetry.addLine();
        telemetry.addLine("Buzzing at centre and never settling means");
        telemetry.addLine("MIN_AIM_POWER overshoots AIM_TOLERANCE_DEG.");

        panels.addData("tx", limelight.hasTarget() ? limelight.getTx() : 0.0);
        panels.addData("hasTarget", limelight.hasTarget());
        panels.addData("onTarget", turret.isOnTarget());
        panels.addData("travelDeg", turret.getTravelDeg());
        panels.addData("unwinding", turret.isUnwinding());
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
