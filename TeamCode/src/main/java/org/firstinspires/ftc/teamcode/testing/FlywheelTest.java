package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuning;

/**
 * Test 3a — spin the flywheel to a set RPM and tune the velocity loop.
 *
 * <h2>Check the two motors agree before you tune anything</h2>
 * The pair face opposite ways, so one is reversed in {@code Constants.Flywheel}.
 * If that reversal is wrong the two motors fight each other: the wheel barely
 * turns, both current readings go through the roof, and the PID — which only
 * reads the LEFT encoder — sees a low speed and pushes harder, making it worse.
 * This test shows both velocities and both currents side by side so you catch
 * that in the first five seconds instead of after a burnt motor. The two
 * velocities should be roughly equal in magnitude and the same sign.
 *
 * <h2>Then check the RPM number is real</h2>
 * {@code TICKS_PER_REV} is set to 28, which is a bare motor with no gearbox. If
 * your flywheel is geared or on a gearboxed motor, every RPM figure in the robot
 * is wrong by that ratio — including the shooting table, which you would then
 * spend an afternoon tuning around a lie. Spin the wheel one full turn by hand
 * from a standstill and confirm the tick count moves by 28.
 *
 * <h2>Then tune</h2>
 * kF first: with kP/kD at zero, raise kF until the wheel settles near the
 * target on its own. Then kP for the remaining gap, then kD if it oscillates.
 * Leave kI at 0 — a flywheel that dips on every shot will wind an integral up
 * and overshoot on the recovery.
 *
 * <p><b>Watch the recovery, not the steady state.</b> The number that decides
 * your fire rate is how fast RPM comes back after a ball goes through, because
 * the shooter refuses to feed until it does. Spin up, fire a ball through by
 * hand, and watch the dip on the Panels graph.
 *
 * <p><b>Controls:</b> <b>A</b> spin up · <b>B</b> stop · bumpers TEST_RPM
 * +/- 100 · dpad up/down TEST_RPM +/- 500.
 */
public class FlywheelTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Flywheel flywheel;
    private boolean spinning = false;

    @Override
    public void init() {
        hardware.initFlywheel(hardwareMap);
        flywheel = new Flywheel(hardware);
        telemetry.addLine("Flywheel test. GUARD ON, nothing loose near the wheel.");
        telemetry.addLine("A = spin up, B = stop.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            spinning = true;
        }
        if (gamepad1.bWasPressed()) {
            spinning = false;
        }
        if (gamepad1.rightBumperWasPressed()) {
            FlywheelTuning.TEST_RPM += 100;
        }
        if (gamepad1.leftBumperWasPressed()) {
            FlywheelTuning.TEST_RPM = Math.max(0, FlywheelTuning.TEST_RPM - 100);
        }
        if (gamepad1.dpadUpWasPressed()) {
            FlywheelTuning.TEST_RPM += 500;
        }
        if (gamepad1.dpadDownWasPressed()) {
            FlywheelTuning.TEST_RPM = Math.max(0, FlywheelTuning.TEST_RPM - 500);
        }

        if (spinning) {
            flywheel.setTargetRpm(FlywheelTuning.TEST_RPM);
        } else {
            flywheel.stop();
        }
        flywheel.periodic();

        // The right motor's own velocity is the tell for a wrong reversal. The
        // control loop never looks at it, so nothing else in the robot would.
        double leftRpm = hardware.flywheelLeft.getVelocity() / Constants.Flywheel.TICKS_PER_REV * 60.0;
        double rightRpm = hardware.flywheelRight.getVelocity() / Constants.Flywheel.TICKS_PER_REV * 60.0;
        double leftAmps = hardware.flywheelLeft.getCurrent(CurrentUnit.AMPS);
        double rightAmps = hardware.flywheelRight.getCurrent(CurrentUnit.AMPS);

        telemetry.addData(">> ", spinning
                ? (flywheel.atTargetRpm() ? "AT SPEED" : "SPINNING UP")
                : "STOPPED  (A to spin up)");
        telemetry.addData(">> Target", "%.0f rpm   (bumpers +/-100, dpad +/-500)",
                FlywheelTuning.TEST_RPM);
        telemetry.addLine();
        telemetry.addData("Left  motor", "%7.0f rpm | %.2f A  <- the PID reads this one", leftRpm, leftAmps);
        telemetry.addData("Right motor", "%7.0f rpm | %.2f A", rightRpm, rightAmps);

        // Same sign and similar magnitude, or the pair is fighting.
        boolean agreeing = Math.abs(leftRpm) < 100
                || (Math.signum(leftRpm) == Math.signum(rightRpm)
                    && Math.abs(Math.abs(rightRpm) - Math.abs(leftRpm)) < Math.abs(leftRpm) * 0.5);
        if (!agreeing) {
            telemetry.addLine("!! MOTORS DISAGREE — stop and flip");
            telemetry.addLine("!! Constants.Flywheel.RIGHT_DIRECTION");
        }
        if (spinning && (leftAmps > 8 || rightAmps > 8)) {
            telemetry.addLine("!! HIGH CURRENT — something is binding or fighting");
        }
        telemetry.addLine();
        telemetry.addData("Ticks/rev assumed", "%.0f  (hand-spin one turn to verify)",
                Constants.Flywheel.TICKS_PER_REV);
        telemetry.addLine("Tune kF first, then kP, then kD. Leave kI at 0.");
        telemetry.addLine("Watch the DIP after a ball, not the steady state.");

        panels.addData("targetRpm", flywheel.getTargetRpm());
        panels.addData("actualRpm", flywheel.getCurrentRpm());
        panels.addData("rightRpm", rightRpm);
        panels.addData("atTarget", flywheel.atTargetRpm());
        panels.addData("leftAmps", leftAmps);
        panels.addData("rightAmps", rightAmps);
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
