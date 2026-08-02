package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Test 6d — build the distance/RPM shooting table by shooting, one measured
 * distance at a time.
 *
 * <p>{@link FlywheelTest} proves the wheel can hold a commanded RPM.
 * {@link ShooterTest} proves the whole chain fires. Neither tells you WHICH RPM
 * scores from three meters out — that is this test, and it is the only one whose
 * output you actually copy into {@code Constants.Flywheel}.
 *
 * <h2>How to run it</h2>
 * <ol>
 *   <li>Park the robot square to the goal and measure the distance with a tape.
 *       Type that number in with <b>Y / X</b> (5 cm a press).</li>
 *   <li><b>Left bumper</b> to rev. Adjust the RPM with the dpad until the shot
 *       scores — up/down is 100, left/right is 25.</li>
 *   <li><b>Right bumper</b> loads a ball, <b>left trigger</b> fires it. The feed
 *       is gated on the wheel being at speed and the turret being locked, so a
 *       ball never leaves during spin-up and skews your result.</li>
 *   <li>Scored twice in a row? Press <b>A</b> to record the point.</li>
 *   <li>Move to the next distance and repeat. Three or four points across your
 *       real shooting range is plenty — the code interpolates between them and
 *       clamps outside.</li>
 * </ol>
 *
 * <p>The bottom of the telemetry is the two array literals, kept sorted and
 * paste-ready for {@code Constants.Flywheel}. They are also written to
 * {@code /sdcard/FIRST/settings/ShootingTable.txt} on every record, so a
 * flat battery between the field and the laptop doesn't cost you the session.
 *
 * <h2>Calibrate the camera FIRST — test 5a</h2>
 * The tape measure is ground truth, but it is not what the robot uses. At match
 * time the table is looked up with {@code Limelight.getDistanceMeters()}, so a
 * table indexed by tape distances is only correct if the camera agrees with the
 * tape. This test shows both numbers and the gap between them, and complains if
 * they diverge. If it does complain, stop, fix the camera geometry in test 5a,
 * and start the table over — otherwise every ranged shot in a match is looked up
 * at the wrong row and you will blame the RPMs you tuned here.
 *
 * <p>The flywheel PIDF (test 3a) also has to be tuned before this means
 * anything. If the wheel can't hold the RPM you asked for, you are not tuning a
 * table, you are tuning noise.
 *
 * <p><b>Controls:</b> INIT screen X blue goal / B red goal. Then <b>Y/X</b>
 * distance +/- 5 cm · <b>dpad up/down</b> RPM +/-100 · <b>dpad left/right</b>
 * RPM +/-25 · <b>left bumper</b> rev · <b>left trigger</b> fire · <b>right
 * bumper</b> intake · <b>right trigger</b> un-jam · <b>A</b> record ·
 * <b>B</b> undo last · left stick/right stick x nudge the robot at 40%.
 */
public class RangeTuningTest extends OpMode {

    private static final double TRIGGER_THRESHOLD = 0.5;
    // Repositioning between measured distances, not driving. Slow on purpose.
    private static final double DRIVE_SCALE = 0.4;
    // Two points closer than this are the same distance, so recording the second
    // replaces the first. rpmForDistance() divides by the gap between adjacent
    // table entries, so duplicate distances would be a divide-by-zero.
    private static final double SAME_DISTANCE_M = 0.01;
    // Past this gap, a table indexed by tape distances won't be looked up at the
    // right row in a match.
    private static final double VISION_DISAGREEMENT_M = 0.15;

    private static final String FILENAME = "ShootingTable.txt";

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Limelight limelight;
    private Turret turret;

    /** One tuned row: what the tape said, what the camera said, what scored. */
    private static final class Point {
        final double rulerM;
        final double visionM;   // -1 when no tag was visible at the moment of recording
        final double rpm;

        Point(double rulerM, double visionM, double rpm) {
            this.rulerM = rulerM;
            this.visionM = visionM;
            this.rpm = rpm;
        }
    }

    private final List<Point> points = new ArrayList<>();

    private int goalTagId = Constants.Vision.DEFAULT_TARGET_TAG;
    private double rulerM = 1.00;
    private double rpm;
    private boolean revving = false;
    private boolean firing = false;
    private String saveStatus = "nothing recorded yet";

    @Override
    public void init() {
        hardware.initDrive(hardwareMap);
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);
        hardware.initTurret(hardwareMap);
        hardware.initLimelight(hardwareMap);

        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);

        scheduler.reset();
        // Vision refreshes before the turret reads it, same as the match.
        scheduler.registerSubsystem(intake, indexer, flywheel, limelight, turret);

        // Start from what the table already claims for this distance, so the
        // first shot of a re-tune tells you how wrong the current table is
        // rather than starting from an arbitrary number.
        rpm = Flywheel.rpmForDistance(rulerM);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            goalTagId = Constants.Vision.BLUE_GOAL_TAG;
        }
        if (gamepad1.bWasPressed()) {
            goalTagId = Constants.Vision.RED_GOAL_TAG;
        }

        telemetry.addData(">> Aiming at", "%s   (X blue / B red)",
                goalTagId == Constants.Vision.BLUE_GOAL_TAG ? "BLUE goal (20)" : "RED goal (24)");
        telemetry.addLine();
        telemetry.addLine("Calibrate the camera in test 5a before trusting this,");
        telemetry.addLine("and tune the flywheel PIDF in 3a before trusting the RPM.");
        telemetry.addLine();
        telemetry.addLine("Tape-measure the distance, type it in with Y/X,");
        telemetry.addLine("adjust RPM on the dpad until it scores, press A.");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.setTargetTagId(goalTagId);
    }

    @Override
    public void loop() {
        // --- Nudge the robot between distances (robot-centric; no navX needed) ---
        drivebase.drive(-gamepad1.left_stick_y * DRIVE_SCALE,
                gamepad1.left_stick_x * DRIVE_SCALE,
                gamepad1.right_stick_x * DRIVE_SCALE);

        // --- The two numbers you're tuning ---
        if (gamepad1.yWasPressed()) {
            rulerM += 0.05;
        }
        if (gamepad1.xWasPressed()) {
            rulerM = Math.max(0, rulerM - 0.05);
        }
        if (gamepad1.dpadUpWasPressed()) {
            rpm += 100;
        }
        if (gamepad1.dpadDownWasPressed()) {
            rpm = Math.max(0, rpm - 100);
        }
        if (gamepad1.dpadRightWasPressed()) {
            rpm += 25;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            rpm = Math.max(0, rpm - 25);
        }

        turret.setState(Turret.State.AUTO_AIM);
        double distance = limelight.getDistanceMeters();

        // The whole point of this test is that the RPM is YOURS. Never call
        // rpmForDistance() here — auto-ranging while tuning the auto-range table
        // means the wheel chases the table you are trying to replace.
        if (gamepad1.leftBumperWasPressed()) {
            revving = !revving;
        }
        if (revving) {
            flywheel.setTargetRpm(rpm);
        } else {
            flywheel.stop();
        }

        // Same arm/sustain latch as the match TeleOp: a strict gate starts the
        // burst, wider bands keep it fed through the dip a ball causes.
        boolean ready = turret.isOnTarget() && flywheel.atTargetRpm();
        boolean fire = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        if (!fire) {
            firing = false;
        } else if (ready) {
            firing = true;
        }
        boolean keepFiring = firing
                && turret.getAimErrorDeg() <= Constants.Turret.KEEP_AIM_TOLERANCE_DEG
                && flywheel.atTargetRpm(Constants.Flywheel.RPM_KEEP_TOLERANCE);

        intake.setState(gamepad1.right_bumper ? Intake.State.INTAKING : Intake.State.IDLE);
        if (keepFiring) {
            indexer.setState(Indexer.State.FEEDING);
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            indexer.setState(Indexer.State.REVERSING);
        } else {
            indexer.setState(Indexer.State.IDLE);
        }

        // --- Record / undo ---
        if (gamepad1.aWasPressed()) {
            record(distance);
        }
        if (gamepad1.bWasPressed() && !points.isEmpty()) {
            points.remove(points.size() - 1);
            save();
        }

        scheduler.run();

        // scheduler.run() just refreshed vision, so re-read rather than printing
        // the pre-refresh value next to a "target visible" that disagrees.
        double shownDistance = limelight.getDistanceMeters();

        telemetry.addLine(status(ready, keepFiring));
        telemetry.addData("Tape distance", "%.2f m   (Y/X +/-5cm)", rulerM);
        telemetry.addData("Target RPM", "%.0f   (dpad up/dn 100, L/R 25)", rpm);
        telemetry.addData("Flywheel", "%.0f rpm%s", flywheel.getCurrentRpm(),
                flywheel.atTargetRpm() ? "  — at speed" : "");
        telemetry.addData("Camera says", shownDistance > 0
                ? String.format("%.2f m   (tape %+.2f m off)", shownDistance, rulerM - shownDistance)
                : "-- no tag");
        telemetry.addLine();
        telemetry.addData("Recorded", "%d points   (A record / B undo)", points.size());
        if (points.isEmpty()) {
            // Never show an empty pair of braces here. Pasting {} into Constants
            // compiles fine and then throws out of rpmForDistance() on the first
            // ranged shot, which is a miserable thing to debug in a match.
            telemetry.addLine("(no points yet — nothing to paste)");
        } else {
            telemetry.addLine(distancesLiteral());
            telemetry.addLine(rpmsLiteral());
        }
        if (worstVisionGap() > VISION_DISAGREEMENT_M) {
            telemetry.addLine("!! camera disagrees with the tape by "
                    + String.format("%.2f m", worstVisionGap()));
            telemetry.addLine("!! fix test 5a and re-record, or the match");
            telemetry.addLine("!! looks this table up at the wrong row");
        }
        telemetry.addData("Saved", saveStatus);
        telemetry.update();
    }

    private String status(boolean ready, boolean keepFiring) {
        if (!revving) {
            return ">> IDLE — LB to rev";
        }
        if (keepFiring) {
            return ">> FIRING";
        }
        if (ready) {
            return ">> READY — hold LT to fire";
        }
        if (!limelight.hasTarget()) {
            return ">> REVVING — no goal in view";
        }
        if (!turret.isOnTarget()) {
            return ">> REVVING — turret still aiming";
        }
        return String.format(">> REVVING — %.0f of %.0f rpm",
                flywheel.getCurrentRpm(), rpm);
    }

    /**
     * Adds the current tape distance and RPM to the table, replacing any point
     * already at that distance, and keeps the list sorted. Both are invariants
     * {@code Flywheel.rpmForDistance()} depends on: it walks the array assuming
     * strictly increasing distances.
     */
    private void record(double visionM) {
        for (int i = points.size() - 1; i >= 0; i--) {
            if (Math.abs(points.get(i).rulerM - rulerM) < SAME_DISTANCE_M) {
                points.remove(i);
            }
        }
        points.add(new Point(rulerM, visionM, rpm));
        Collections.sort(points, (a, b) -> Double.compare(a.rulerM, b.rulerM));
        save();
    }

    private String distancesLiteral() {
        StringBuilder sb = new StringBuilder("RANGE_DISTANCES_M = {");
        for (int i = 0; i < points.size(); i++) {
            sb.append(i == 0 ? "" : ", ").append(String.format("%.2f", points.get(i).rulerM));
        }
        return sb.append("};").toString();
    }

    private String rpmsLiteral() {
        StringBuilder sb = new StringBuilder("RANGE_RPMS        = {");
        for (int i = 0; i < points.size(); i++) {
            sb.append(i == 0 ? "" : ", ").append(String.format("%.0f", points.get(i).rpm));
        }
        return sb.append("};").toString();
    }

    /** Largest tape-vs-camera gap across the recorded points; 0 if none seen. */
    private double worstVisionGap() {
        double worst = 0;
        for (Point p : points) {
            if (p.visionM > 0) {
                worst = Math.max(worst, Math.abs(p.rulerM - p.visionM));
            }
        }
        return worst;
    }

    /**
     * Mirrors the table to the Robot Controller after every change. Overwrites
     * rather than appends, so the file is always exactly the current table and
     * never a pile of half-finished runs you have to reconcile by hand.
     */
    private void save() {
        StringBuilder sb = new StringBuilder();
        sb.append("Shooting table — paste into Constants.Flywheel\n");
        sb.append("Rewritten on every record; this is the whole table.\n\n");
        sb.append(distancesLiteral()).append('\n');
        sb.append(rpmsLiteral()).append("\n\n");
        sb.append("tape_m,camera_m,gap_m,rpm\n");
        for (Point p : points) {
            sb.append(String.format("%.2f,%s,%s,%.0f\n", p.rulerM,
                    p.visionM > 0 ? String.format("%.2f", p.visionM) : "",
                    p.visionM > 0 ? String.format("%+.2f", p.rulerM - p.visionM) : "",
                    p.rpm));
        }

        try {
            File file = AppUtil.getInstance().getSettingsFile(FILENAME);
            ReadWriteFile.writeFile(file, sb.toString());
            saveStatus = String.format("%d pts -> %s", points.size(), FILENAME);
        } catch (RuntimeException e) {
            // A failed write must not cost you the session — the table is still
            // on screen, so say so rather than dying mid-tune.
            saveStatus = "FILE WRITE FAILED — copy off the screen";
        }
    }

    @Override
    public void stop() {
        scheduler.cancelAll();
        drivebase.stop();
        flywheel.stop();
        flywheel.periodic();
        turret.stop();
        hardware.limelight.stop();
    }
}
