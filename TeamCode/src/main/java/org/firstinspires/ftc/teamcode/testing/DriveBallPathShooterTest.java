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
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.tuning.FlywheelTuning;

/**
 * Test 6c — field-relative driving with the intake, indexer and flywheel live,
 * and nothing that aims itself.
 *
 * <h2>Why this test exists</h2>
 * Test 3c proves the ball path and the flywheel can all run at once while the
 * robot sits still; test 1b proves the drivebase and the navX. This is the two
 * of them at the same time, which is the first time eight motors pull off one
 * battery. That is where a brownout actually shows up: a flywheel holding RPM
 * on blocks can still drop the hub when the drivebase accelerates under it.
 * Watch the <b>battery</b> line while you drive hard, not just while you spin up.
 *
 * <p>It is also the first test a driver can practise on. Every other subsystem
 * is out of the loop, so nothing moves that you did not ask for.
 *
 * <h2>What is deliberately NOT here</h2>
 * No <b>turret</b>, no <b>auto-aim</b>, no <b>Limelight</b>, no <b>hood</b>, no
 * <b>stopper</b>. None of them are initialized, so this test runs with any of
 * them unplugged or half-built, and nothing can swing while you are driving.
 *
 * <p><b>There is no gate.</b> The stopper is unpowered, so the indexer feeds
 * straight into the flywheel — every ball you index leaves the robot at whatever
 * speed the wheel happens to be at. Point it somewhere safe. The <b>hood</b> is
 * unpowered too, so the ball leaves at whatever angle the hood is physically
 * resting at; where the shot lands is not this test's business. Judging the shot
 * is 6a.
 *
 * <h2>What to prove</h2>
 * <ul>
 *   <li><b>Field-relative holds while the shooter runs.</b> Zero the heading
 *       pointing downfield, then drive a lap and come back: forward on the stick
 *       must still mean downfield. A heading that drifts once the flywheel is
 *       spinning is the navX picking up vibration or a loose mount.</li>
 *   <li><b>The battery survives both loads.</b> Spin up, then accelerate and
 *       stop hard. Anything under 11 V and the shooting table you tune later
 *       will not hold at the end of a match.</li>
 *   <li><b>The flywheel recovers its RPM after a drive input.</b> Watch how long
 *       "AT SPEED" takes to come back after you slam the sticks.</li>
 *   <li><b>Driving does not jam the ball path.</b> Run intake and indexer while
 *       driving over the field seams and watch the ball-path currents.</li>
 * </ul>
 *
 * <p><b>Controls:</b> left stick translate · right stick X turn · <b>options</b>
 * zero the heading · <b>X</b> field/robot centric · hold <b>B</b> for precision
 * speed · <b>A</b> toggles the flywheel · dpad up/down ramp the target RPM ·
 * dpad left/right nudge it ±25 · <b>Y</b> snaps it back to the default shot ·
 * <b>right bumper</b> intake in · <b>right trigger</b> intake out · <b>left
 * bumper</b> indexer feed · <b>left trigger</b> indexer reverse. Only the
 * flywheel latches; let go of anything else and it stops.
 */
public class DriveBallPathShooterTest extends OpMode {

    private static final double TRIGGER_THRESHOLD = 0.5;

    // Held-B speed. Slow enough to line up on a goal without a stick fight.
    private static final double PRECISION_SCALE = 0.35;

    // How fast dpad up/down ramps the RPM target when held.
    private static final double RPM_PER_SECOND = 1000.0;

    // Single-press step on dpad left/right, matched to the at-speed band so one
    // press is the smallest change that can move you in or out of tolerance.
    private static final double RPM_STEP = 25.0;

    // Spin-up surge is brief; a sustained draw this high on one motor is binding.
    private static final double HIGH_CURRENT_AMPS = 8.0;

    // Below this the hub is close to browning out and no reading is trustworthy.
    private static final double LOW_VOLTAGE = 11.0;

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private VoltageSensor battery;

    // Field-relative is the point of the test, so start there. X drops to
    // robot-centric if the navX is lying and you want to keep driving anyway.
    private boolean fieldCentric = true;
    private boolean spinning = false;

    // Worst voltage seen since INIT. The sag that matters happens during a
    // half-second spin-up or a hard stop, which is easy to miss on a live number.
    private double minVolts = Double.MAX_VALUE;

    @Override
    public void init() {
        // Drive + navX + ball path + flywheel, and nothing else. No turret, hood,
        // stopper or camera, so a missing one of those can't stop this running.
        hardware.initDrive(hardwareMap);
        hardware.initNavx(hardwareMap);
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);

        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);

        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Drive + ball path + flywheel. FIELD-CENTRIC.");
        telemetry.addLine("initNavx zeroed the heading at INIT: point the robot");
        telemetry.addLine("downfield now, or press options once you have.");
        telemetry.addLine("NO gate and NO hood — a fed ball goes straight out.");
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

        // --- Drive. Both sticks belong to the drivebase, which is why the RPM
        // controls live on the dpad in this test and on a stick in 3c. ---
        if (gamepad1.xWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.optionsWasPressed()) {
            // Whatever the robot is pointing at right now becomes "forward" for
            // the stick. Do this with the robot squared up downfield.
            drivebase.resetHeading();
        }

        double scale = gamepad1.b ? PRECISION_SCALE : 1.0;
        double axial = -gamepad1.left_stick_y * scale;
        double lateral = gamepad1.left_stick_x * scale;
        double yaw = gamepad1.right_stick_x * scale;

        if (fieldCentric) {
            drivebase.driveFieldCentric(axial, lateral, yaw);
        } else {
            drivebase.drive(axial, lateral, yaw);
        }

        // --- Flywheel: the only latched control, so you can spin up and then
        // put both thumbs back on the sticks. ---
        if (gamepad1.aWasPressed()) {
            spinning = !spinning;
        }
        double rpmChange = 0;
        if (gamepad1.dpad_up) {
            rpmChange += RPM_PER_SECOND * dt;
        }
        if (gamepad1.dpad_down) {
            rpmChange -= RPM_PER_SECOND * dt;
        }
        if (gamepad1.dpadRightWasPressed()) {
            rpmChange += RPM_STEP;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            rpmChange -= RPM_STEP;
        }
        if (rpmChange != 0) {
            FlywheelTuning.TEST_RPM = Range.clip(FlywheelTuning.TEST_RPM + rpmChange,
                    0, Constants.Flywheel.MAX_RPM);
        }
        if (gamepad1.yWasPressed()) {
            FlywheelTuning.TEST_RPM = Constants.Flywheel.SHOOT_RPM;
        }
        if (spinning) {
            flywheel.setTargetRpm(FlywheelTuning.TEST_RPM);
        } else {
            flywheel.stop();
        }

        // --- Intake and indexer: held, never latched. Nothing gates the indexer,
        // so a feed press is a shot. ---
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intake.setState(Intake.State.INTAKING);
        } else if (gamepad1.right_bumper) {
            intake.setState(Intake.State.OUTTAKING);
        } else {
            intake.setState(Intake.State.IDLE);
        }

        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            indexer.setState(Indexer.State.FEEDING);
        } else if (gamepad1.left_bumper) {
            indexer.setState(Indexer.State.REVERSING);
        } else {
            indexer.setState(Indexer.State.IDLE);
        }

        intake.periodic();
        indexer.periodic();
        flywheel.periodic();

        // --- Readings. Sample each device once so every line below agrees. ---
        double leftRpm = hardware.flywheelLeft.getVelocity()
                / Constants.Flywheel.TICKS_PER_REV * 60.0;
        double rightRpm = hardware.flywheelRight.getVelocity()
                / Constants.Flywheel.TICKS_PER_REV * 60.0;
        double leftAmps = hardware.flywheelLeft.getCurrent(CurrentUnit.AMPS);
        double rightAmps = hardware.flywheelRight.getCurrent(CurrentUnit.AMPS);
        double intakeAmps = hardware.intake.getCurrent(CurrentUnit.AMPS);
        double indexerAmps = hardware.indexer.getCurrent(CurrentUnit.AMPS);
        double driveAmps = hardware.frontLeft.getCurrent(CurrentUnit.AMPS)
                + hardware.frontRight.getCurrent(CurrentUnit.AMPS)
                + hardware.backLeft.getCurrent(CurrentUnit.AMPS)
                + hardware.backRight.getCurrent(CurrentUnit.AMPS);
        double volts = battery.getVoltage();
        // The sensor reads 0 while the hub is mid-refresh; that isn't a real sag.
        if (volts > 0) {
            minVolts = Math.min(minVolts, volts);
        }

        telemetry.addData(">> ", spinning
                ? (flywheel.atTargetRpm() ? "AT SPEED" : "SPINNING UP")
                : "FLYWHEEL OFF  (A to spin up)");
        telemetry.addData(">> Target", "%.0f rpm   (dpad U/D ramp, L/R +/-%.0f, Y = %.0f)",
                FlywheelTuning.TEST_RPM, RPM_STEP, Constants.Flywheel.SHOOT_RPM);
        telemetry.addLine();

        telemetry.addLine("--- drive ---");
        telemetry.addData("Mode", "%s   (X to switch)",
                fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Speed", "%.0f%%   (hold B for precision)", scale * 100);
        telemetry.addData("Heading", "%+.1f deg   (options re-zeros it here)",
                Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("FL / FR", "%+.2f  %+.2f",
                hardware.frontLeft.getPower(), hardware.frontRight.getPower());
        telemetry.addData("BL / BR", "%+.2f  %+.2f",
                hardware.backLeft.getPower(), hardware.backRight.getPower());

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
        telemetry.addLine("--- ball path (no gate: a feed IS a shot) ---");
        telemetry.addData("Intake", "%-9s %+.2f | %.2f A   (RB in / RT out)",
                intake.getState(), hardware.intake.getPower(), intakeAmps);
        telemetry.addData("Indexer", "%-9s %+.2f | %.2f A   (LB feed / LT reverse)",
                indexer.getState(), hardware.indexer.getPower(), indexerAmps);

        telemetry.addLine();
        telemetry.addLine("--- everything at once ---");
        telemetry.addData("Battery", "%.2f V   (lowest since init %s)", volts,
                minVolts == Double.MAX_VALUE ? "--" : String.format("%.2f V", minVolts));
        telemetry.addData("Total draw", "%.2f A   (drive %.2f A)",
                leftAmps + rightAmps + intakeAmps + indexerAmps + driveAmps, driveAmps);
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
        telemetry.addLine("Drive hard WHILE the wheel is at speed. Both loads on");
        telemetry.addLine("one battery is the thing this test is here to find.");

        panels.addData("targetRpm", flywheel.getTargetRpm());
        panels.addData("leftRpm", leftRpm);
        panels.addData("rightRpm", rightRpm);
        panels.addData("atTarget", flywheel.atTargetRpm());
        panels.addData("leftAmps", leftAmps);
        panels.addData("rightAmps", rightAmps);
        panels.addData("intakeAmps", intakeAmps);
        panels.addData("indexerAmps", indexerAmps);
        panels.addData("driveAmps", driveAmps);
        panels.addData("volts", volts);
        panels.addData("headingDeg", Math.toDegrees(drivebase.getHeading()));
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        drivebase.stop();
        intake.setState(Intake.State.IDLE);
        indexer.setState(Indexer.State.IDLE);
        flywheel.stop();

        intake.periodic();
        indexer.periodic();
        flywheel.periodic();
    }
}
