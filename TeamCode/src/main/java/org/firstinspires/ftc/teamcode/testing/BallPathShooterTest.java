package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuning;
import org.firstinspires.ftc.teamcode.tuning.HoodTuning;

/**
 * Test 3c — intake, indexer, both flywheels and the hood, all in one OpMode,
 * with the two servos that can hurt you left out.
 *
 * <h2>Why this test exists</h2>
 * Tests 2a, 3a and 3b each prove one mechanism in isolation, which is the right
 * way to find a miswired motor but says nothing about what happens when they all
 * pull at once. Four motors spinning up together is where you find the problems
 * that only show up under load: a battery that sags far enough to reset the hub,
 * a hub current limit, an intake that stalls only while the flywheel is drawing
 * its spin-up surge. So run this once everything upstream passes and watch the
 * <b>battery voltage</b> line as much as the mechanisms.
 *
 * <h2>What is deliberately NOT here</h2>
 * The <b>stopper</b> gate and the <b>turret</b> rotation servo are not
 * initialized and never commanded, so this test is safe to run with either of
 * them unplugged, half-built, or past a stop you haven't found yet. The hood
 * servo <i>is</i> here, but only under manual control and still clamped to the
 * {@code HoodTuning} band.
 *
 * <p><b>The gate is unpowered, not open.</b> Nothing commands the stopper, so if
 * it is bolted on it sits wherever it was left, with no holding torque. Feeding
 * the indexer may push a ball straight into the flywheel or may jam against a
 * dead gate — either is expected here. Judging the gate's timing is test 2b, and
 * the whole aim-rev-fire chain is 6a.
 *
 * <h2>What to prove</h2>
 * <ul>
 *   <li><b>Both flywheel motors agree.</b> Same sign, similar magnitude. The PID
 *       only reads the left encoder, so a wrong {@code RIGHT_DIRECTION} shows up
 *       as the pair fighting: low speed, both currents high, and the loop pushing
 *       harder because it thinks the wheel is slow.</li>
 *   <li><b>The flywheel holds its RPM while the ball path runs.</b> Spin up, then
 *       run intake and indexer together. A target the wheel held on its own but
 *       loses with everything else moving is a power problem, not a PID one.</li>
 *   <li><b>Battery voltage stays up.</b> Under 11 V while spinning up means the
 *       shooting table you tune later won't hold at the end of a match.</li>
 *   <li><b>Intake pulls in, indexer feeds toward the flywheel.</b> If either runs
 *       backwards, flip {@code DIRECTION} in {@code Constants} — not the
 *       buttons, because autonomous calls the same enums.</li>
 *   <li><b>The hood moves and holds under vibration.</b> Jog it while the
 *       flywheel is at speed; it should stay put.</li>
 * </ul>
 *
 * <p><b>Controls:</b> <b>A</b> toggles the flywheel · right stick up/down ramps
 * the target RPM · <b>right bumper</b> intake in · <b>right trigger</b> intake
 * out · <b>left bumper</b> indexer feed · <b>left trigger</b> indexer reverse ·
 * dpad left/right hood ∓0.01 · dpad up/down hood ±0.05 · <b>X</b> near preset ·
 * <b>B</b> far preset · <b>Y</b> cuts/restores hood power. Only the flywheel
 * and the hood power latch; let go of anything else and it stops.
 *
 * <h2>The hood starts limp, on purpose</h2>
 * Nothing energizes the hood servo until you press <b>Y</b>, and <b>Y</b> again
 * cuts it dead — a real PWM cut, so you can move the hood by hand and so a servo
 * buzzing against a stop can be killed from the gamepad instead of by
 * power-cycling the robot. When you do arm it, it comes up in the middle of the
 * travel band rather than at {@code DEFAULT_POSITION}, which is defined as
 * {@code MIN_POSITION}: a hood parked exactly on the bottom clamp ignores every
 * downward nudge and reads as broken.
 */
public class BallPathShooterTest extends OpMode {

    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double STICK_THRESHOLD = 0.2;

    // How fast the right stick ramps the RPM target when held all the way over.
    private static final double RPM_PER_SECOND = 1000.0;

    // Spin-up surge is brief; a sustained draw this high on one motor is binding.
    private static final double HIGH_CURRENT_AMPS = 8.0;

    // Below this the hub is close to browning out and no reading is trustworthy.
    private static final double LOW_VOLTAGE = 11.0;

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Hood hood;
    private VoltageSensor battery;

    private boolean spinning = false;

    // Worst voltage seen since INIT. The sag that matters happens during a
    // half-second spin-up, which is easy to miss on a live number.
    private double minVolts = Double.MAX_VALUE;

    @Override
    public void init() {
        // Only the four groups this test drives. No stopper, no turret — so a
        // missing or half-built one of those can't stop the test from running.
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);
        hardware.initHood(hardwareMap);

        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        hood = new Hood(hardware);

        battery = hardwareMap.voltageSensor.iterator().next();

        // Hood starts LIMP. Nothing energizes it until you press Y, so you can
        // move it by hand first and see where the linkage actually reaches.
        hood.setPwmEnabled(false);

        // When you do arm it, come up in the MIDDLE of the travel band, not at
        // DEFAULT_POSITION. DEFAULT_POSITION is defined as MIN_POSITION, so a
        // hood parked there sits exactly on the bottom clamp: every downward
        // nudge clips straight back to the same number and the servo never
        // moves, which reads as a dead hood. From the midpoint both directions
        // work and you are as far as possible from either stop.
        HoodTuning.TEST_POSITION = (HoodTuning.MIN_POSITION + HoodTuning.MAX_POSITION) / 2.0;
        hood.setPosition(HoodTuning.TEST_POSITION);

        telemetry.addLine("Ball path + shooter together. GUARD ON.");
        telemetry.addLine("Stopper and turret are NOT powered in this test.");
        telemetry.addLine("Hood is LIMP until you press Y — move it by hand first.");
        telemetry.addLine("A = flywheel, RB = intake, LB = indexer feed.");
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // --- Flywheel: the only latched control, so you can spin up and then
        // use both hands on the ball path. ---
        if (gamepad1.aWasPressed()) {
            spinning = !spinning;
        }
        // Stick forward is negative on a gamepad, so up means faster.
        if (Math.abs(gamepad1.right_stick_y) > STICK_THRESHOLD) {
            FlywheelTuning.TEST_RPM = Range.clip(
                    FlywheelTuning.TEST_RPM - gamepad1.right_stick_y * RPM_PER_SECOND * dt,
                    0, Constants.Flywheel.MAX_RPM);
        }
        if (spinning) {
            flywheel.setTargetRpm(FlywheelTuning.TEST_RPM);
        } else {
            flywheel.stop();
        }

        // --- Intake and indexer: held, never latched. ---
        if (gamepad1.right_bumper) {
            intake.setState(Intake.State.INTAKING);
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intake.setState(Intake.State.OUTTAKING);
        } else {
            intake.setState(Intake.State.IDLE);
        }

        if (gamepad1.left_bumper) {
            indexer.setState(Indexer.State.FEEDING);
        } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            indexer.setState(Indexer.State.REVERSING);
        } else {
            indexer.setState(Indexer.State.IDLE);
        }

        // --- Hood: manual only. There is no camera in this test, so nothing to
        // auto-range off. Y is the power cut: it goes limp on the spot, which is
        // what you hit the moment the servo starts buzzing. Nudges still track
        // while it is off, so you can line a position up before energizing. ---
        if (gamepad1.yWasPressed()) {
            hood.setPwmEnabled(!hood.isPwmEnabled());
        }

        double nudge = 0;
        if (gamepad1.dpadRightWasPressed()) {
            nudge = 0.01;
        }
        if (gamepad1.dpadLeftWasPressed()) {
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

        intake.periodic();
        indexer.periodic();
        flywheel.periodic();
        hood.periodic();

        // --- Readings. Sample each device once so every line below agrees. ---
        double leftRpm = hardware.flywheelLeft.getVelocity()
                / Constants.Flywheel.TICKS_PER_REV * 60.0;
        double rightRpm = hardware.flywheelRight.getVelocity()
                / Constants.Flywheel.TICKS_PER_REV * 60.0;
        double leftAmps = hardware.flywheelLeft.getCurrent(CurrentUnit.AMPS);
        double rightAmps = hardware.flywheelRight.getCurrent(CurrentUnit.AMPS);
        double intakeAmps = hardware.intake.getCurrent(CurrentUnit.AMPS);
        double indexerAmps = hardware.indexer.getCurrent(CurrentUnit.AMPS);
        double volts = battery.getVoltage();
        // The sensor reads 0 while the hub is mid-refresh; that isn't a real sag.
        if (volts > 0) {
            minVolts = Math.min(minVolts, volts);
        }

        telemetry.addData(">> ", spinning
                ? (flywheel.atTargetRpm() ? "AT SPEED" : "SPINNING UP")
                : "FLYWHEEL OFF  (A to spin up)");
        telemetry.addData(">> Target", "%.0f rpm   (right stick up/down)",
                FlywheelTuning.TEST_RPM);
        telemetry.addLine();

        telemetry.addLine("--- flywheel: the two must agree ---");
        telemetry.addData("Left  motor", "%7.0f rpm | %.2f A  <- the PID reads this one",
                leftRpm, leftAmps);
        telemetry.addData("Right motor", "%7.0f rpm | %.2f A", rightRpm, rightAmps);
        // Same sign and similar magnitude, or the pair is fighting each other.
        boolean agreeing = Math.abs(leftRpm) < 100
                || (Math.signum(leftRpm) == Math.signum(rightRpm)
                    && Math.abs(Math.abs(rightRpm) - Math.abs(leftRpm)) < Math.abs(leftRpm) * 0.5);
        if (!agreeing) {
            telemetry.addLine("!! MOTORS DISAGREE — stop and flip");
            telemetry.addLine("!! Constants.Flywheel.RIGHT_DIRECTION");
        }

        telemetry.addLine();
        telemetry.addLine("--- ball path ---");
        telemetry.addData("Intake", "%-9s %+.2f | %.2f A   (RB in / RT out)",
                intake.getState(), hardware.intake.getPower(), intakeAmps);
        telemetry.addData("Indexer", "%-9s %+.2f | %.2f A   (LB feed / LT reverse)",
                indexer.getState(), hardware.indexer.getPower(), indexerAmps);

        telemetry.addLine();
        telemetry.addLine("--- hood (manual; no camera in this test) ---");
        boolean clamped = Math.abs(hood.getTargetPosition() - hood.getCommandedPosition()) > 1e-6;
        telemetry.addData("Power", hood.isPwmEnabled()
                ? "LIVE — Y cuts it (do that if it buzzes)"
                : "LIMP — move it by hand. Y energizes it.");
        telemetry.addData("Commanded", "%.3f%s", hood.getCommandedPosition(),
                clamped ? String.format("  (CLAMPED — asked for %.3f)", hood.getTargetPosition()) : "");
        telemetry.addData("Safe band", "%.3f .. %.3f   dpad L/R +/-0.01, U/D +/-0.05",
                HoodTuning.MIN_POSITION, HoodTuning.MAX_POSITION);
        telemetry.addData("Presets", "near %.3f (X)   far %.3f (B)",
                HoodTuning.NEAR_PRESET, HoodTuning.FAR_PRESET);
        if (clamped) {
            telemetry.addLine("   Sitting on a clamp — nudges this way do nothing.");
            telemetry.addLine("   Widen the band in test 3b, don't force it here.");
        }

        telemetry.addLine();
        telemetry.addLine("--- everything at once ---");
        telemetry.addData("Battery", "%.2f V   (lowest since init %s)", volts,
                minVolts == Double.MAX_VALUE ? "--" : String.format("%.2f V", minVolts));
        telemetry.addData("Total draw", "%.2f A",
                leftAmps + rightAmps + intakeAmps + indexerAmps);
        if (minVolts < LOW_VOLTAGE) {
            telemetry.addLine("!! BATTERY SAGGED under " + (int) LOW_VOLTAGE
                    + "V — swap it before tuning anything");
        }
        if (spinning && (leftAmps > HIGH_CURRENT_AMPS || rightAmps > HIGH_CURRENT_AMPS)) {
            telemetry.addLine("!! HIGH FLYWHEEL CURRENT — binding or fighting");
        }
        if (intakeAmps > HIGH_CURRENT_AMPS || indexerAmps > HIGH_CURRENT_AMPS) {
            telemetry.addLine("!! HIGH BALL-PATH CURRENT — something is jammed");
        }

        telemetry.addLine();
        telemetry.addLine("Hold the RPM with intake AND indexer running — that");
        telemetry.addLine("is the load the match actually puts on the battery.");

        panels.addData("targetRpm", flywheel.getTargetRpm());
        panels.addData("leftRpm", leftRpm);
        panels.addData("rightRpm", rightRpm);
        panels.addData("atTarget", flywheel.atTargetRpm());
        panels.addData("leftAmps", leftAmps);
        panels.addData("rightAmps", rightAmps);
        panels.addData("intakeAmps", intakeAmps);
        panels.addData("indexerAmps", indexerAmps);
        panels.addData("volts", volts);
        panels.addData("hoodCommanded", hood.getCommandedPosition());
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        intake.setState(Intake.State.IDLE);
        indexer.setState(Indexer.State.IDLE);
        flywheel.stop();
        // Leave the hood limp rather than stowing it. Stow is DEFAULT_POSITION,
        // and if that is the position that stalls the servo, driving to it on
        // the way out re-creates the fault every time you end the test.
        hood.setPwmEnabled(false);

        intake.periodic();
        indexer.periodic();
        flywheel.periodic();
        hood.periodic();
    }
}
