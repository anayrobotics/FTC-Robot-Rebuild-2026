package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SetIndexerStateCommand;
import org.firstinspires.ftc.teamcode.commands.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Single-driver TeleOp: the whole robot on gamepad1, no operator.
 *
 * <p>This class holds the behaviour and carries no {@code @TeleOp} annotation.
 * Alliance is chosen by picking the opmode on the driver station — {@link
 * BlueTeleOp} or {@link RedTeleOp} — and the only difference between them is
 * which goal AprilTag the turret hunts for.
 *
 * <h2>The shooting flow</h2>
 * Aiming is not something the driver does. The turret hunts the goal tag from
 * the moment the opmode starts and never stops, and the hood re-ranges itself
 * off the Limelight every loop, so the robot is always pointed and angled at
 * the goal while you drive. That leaves the driver exactly two decisions:
 * <ol>
 *   <li><b>Left bumper</b> — rev up. The flywheel spins to the RPM for the
 *       measured distance and keeps tracking it as you move.</li>
 *   <li>Watch telemetry. It reads {@code REVVING} while it spins or aims, and
 *       flips to <b>READY TO SHOOT</b> once the turret is locked on AND the
 *       wheel is at speed.</li>
 *   <li><b>Left trigger</b> — fire. Held, it opens the gate and feeds. If the
 *       lock or the speed drops mid-burst the gate shuts on its own, so a shot
 *       that would miss never leaves the robot.</li>
 * </ol>
 * Left bumper again spins the wheel back down.
 *
 * <h2>Controls — gamepad1 only</h2>
 * <ul>
 *   <li><b>Left stick</b> — translate. Always field-relative.</li>
 *   <li><b>Right stick X</b> — turn.</li>
 *   <li><b>Dpad up</b> — re-zero field-forward. Point the robot downfield and
 *       press, at the start and any time the heading drifts.</li>
 *   <li><b>Left bumper</b> — toggle the flywheel rev.</li>
 *   <li><b>Left trigger</b> — hold to FIRE (gated on READY).</li>
 *   <li><b>Right bumper</b> — toggle the intake on/off.</li>
 *   <li><b>Right trigger</b> — spit out, while held.</li>
 *   <li><b>A</b> — toggle the indexer on/off, to load up.</li>
 *   <li><b>B</b> — reverse the indexer while held, to back a jam out.</li>
 *   <li><b>Dpad left / right</b> — override auto-aim and nudge the turret by
 *       hand while held; it resumes hunting the goal on release.</li>
 * </ul>
 *
 * <p>REAL-LIFE WARNING: because the turret now tracks continuously rather than
 * only while a button is held, it can chase the goal round in circles as you
 * drive. It has no travel limit in software — a mechanical hard stop or a slip
 * ring is what keeps it from twisting the wiring off.
 */
public abstract class MecanumTeleOp extends OpMode {
    // Trigger past this counts as "held".
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Hood hood;
    private Stopper stopper;
    private Limelight limelight;
    private Turret turret;

    // Latched by the left bumper: is the shooter spun up and tracking range?
    private boolean revving = false;
    // Latched by the right bumper. The spit-out trigger outranks it while held
    // without clearing it, so releasing the trigger resumes intaking.
    private boolean intakeLatched = false;
    // Latched by A, same idea — outranked by the fire trigger and by B.
    private boolean indexerLatched = false;

    // Last state actually commanded. Both desired states are recomputed from
    // scratch every loop, so these keep us from re-scheduling the same command
    // fifty times a second.
    private Intake.State lastIntakeState = Intake.State.IDLE;
    private Indexer.State lastIndexerState = Indexer.State.IDLE;

    /** Which goal AprilTag this alliance's turret aims at. */
    protected abstract int goalTagId();

    @Override
    public void init(){
        hardware.init(hardwareMap);
        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        hood = new Hood(hardware);
        stopper = new Stopper(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);

        // Set once here instead of on a button, so there's no way to start a
        // match aimed at the other alliance's goal.
        limelight.setTargetTagId(goalTagId());

        scheduler.reset();
        // Order matters: the Limelight must refresh BEFORE the Turret reads it,
        // so the turret aims on this loop's fresh vision data.
        scheduler.registerSubsystem(intake, indexer, flywheel, hood, stopper, limelight, turret);

        telemetry.addLine("Initialized — single driver, gamepad1.");
        telemetry.addData("Aiming at", targetName());
        telemetry.addLine("Turret and hood aim themselves. LB revs, LT fires when READY.");
        telemetry.addLine("RB intake toggle / RT spit  |  A indexer toggle / B reverse");
        telemetry.addLine("Dpad up re-zeroes heading  |  dpad L-R nudges the turret");
        telemetry.update();
    }

    @Override
    public void loop(){
        // --- Driving ---
        if (gamepad1.dpadUpWasPressed()) {
            drivebase.resetHeading();
        }
        drivebase.driveWithGamepad(gamepad1, true);

        // --- Turret: always hunting the goal ---
        // No aim button. Dpad left/right takes it over while held (for lining up
        // by eye if vision is out), and it goes straight back to tracking.
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            turret.setState(Turret.State.MANUAL);
            turret.setManualPower(gamepad1.dpad_left
                    ? -Constants.Turret.MANUAL_NUDGE_POWER
                    : Constants.Turret.MANUAL_NUDGE_POWER);
        } else {
            turret.setState(Turret.State.AUTO_AIM);
        }

        // --- Hood: always ranging ---
        // Any time the Limelight has a range read the hood tracks it; with no
        // read it holds its last angle.
        double distance = limelight.getDistanceMeters();
        if (distance > 0) {
            hood.setForDistance(distance);
        }

        // --- Flywheel: left bumper arms it ---
        // While revving, the target RPM follows the measured distance every loop,
        // so walking toward or away from the goal re-ranges the shot by itself.
        // No range read yet just means the preset.
        if (gamepad1.leftBumperWasPressed()) {
            revving = !revving;
        }
        if (revving) {
            flywheel.setTargetRpm(distance > 0
                    ? Flywheel.rpmForDistance(distance)
                    : Constants.Flywheel.SHOOT_RPM);
        } else {
            flywheel.stop();
        }

        // Aimed, up to speed, and the driver asked for it: the one condition the
        // whole shot is gated on. atTargetRpm() is false at a zero target, so a
        // stopped wheel can never read READY.
        boolean ready = turret.isOnTarget() && flywheel.atTargetRpm();
        boolean fire = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // --- Intake: latching bumper, momentary spit ---
        if (gamepad1.rightBumperWasPressed()) {
            intakeLatched = !intakeLatched;
        }
        Intake.State wantIntake;
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            wantIntake = Intake.State.OUTTAKING;
        } else {
            wantIntake = intakeLatched ? Intake.State.INTAKING : Intake.State.IDLE;
        }
        if (wantIntake != lastIntakeState) {
            scheduler.schedule(new SetIntakeStateCommand(intake, wantIntake));
            lastIntakeState = wantIntake;
        }

        // --- Indexer + gate ---
        // Three claims on one motor, highest priority first: the fire trigger,
        // a held B to back a jam out, then the A toggle for loading up.
        if (gamepad1.aWasPressed()) {
            indexerLatched = !indexerLatched;
        }
        Indexer.State wantIndexer;
        if (fire && ready) {
            // Clear the gate first, and only feed once it has had time to
            // actually swing open — the servo has no position feedback, so
            // feeding immediately would ram a ball into a half-open stopper.
            stopper.open();
            wantIndexer = stopper.isSettled() ? Indexer.State.FEEDING : Indexer.State.IDLE;
        } else {
            // Not firing, or aim/speed dropped mid-burst: hold the next ball
            // back. Pausing the feed beats letting a shot go that would miss.
            stopper.block();
            if (fire) {
                wantIndexer = Indexer.State.IDLE;
            } else if (gamepad1.b) {
                wantIndexer = Indexer.State.REVERSING;
            } else {
                wantIndexer = indexerLatched ? Indexer.State.FEEDING : Indexer.State.IDLE;
            }
        }
        if (wantIndexer != lastIndexerState) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, wantIndexer));
            lastIndexerState = wantIndexer;
        }

        scheduler.run();

        // Shooter state first and loudest — this line is what the driver is
        // actually watching, and it says what to do next rather than making them
        // infer it from four separate readouts.
        telemetry.addLine(shooterStatus(ready, fire));
        telemetry.addLine();
        telemetry.addData("Aiming at", targetName());
        telemetry.addData("Target visible", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
            telemetry.addData("Distance (m)", "%.2f", distance);
        }
        telemetry.addData("Turret", "%s%s", turret.getState(),
                turret.isOnTarget() ? " — LOCKED" : "");
        telemetry.addData("Flywheel", "%.0f / %.0f rpm%s",
                flywheel.getCurrentRpm(), flywheel.getTargetRpm(),
                flywheel.atTargetRpm() ? " — at speed" : "");
        telemetry.addData("Hood position", "%.2f", hood.getCommandedPosition());
        telemetry.addData("Stopper", stopper.getState());
        telemetry.addLine();
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("Intake", "%s%s", intake.getState(), intakeLatched ? " (latched)" : "");
        telemetry.addData("Indexer", "%s%s", indexer.getState(), indexerLatched ? " (latched)" : "");
        telemetry.update();
    }

    // One line telling the driver where the shot is and what to press. Ordered
    // by what's blocking the shot, most fundamental first.
    private String shooterStatus(boolean ready, boolean fire) {
        if (!revving) {
            return ">> IDLE — press LB to rev up";
        }
        if (ready) {
            return fire ? ">> FIRING" : ">> READY TO SHOOT — hold LT to fire";
        }
        if (!limelight.hasTarget()) {
            return ">> REVVING — no goal in view, drive until the tag shows";
        }
        if (!turret.isOnTarget()) {
            return ">> REVVING — aiming...";
        }
        return String.format(">> REVVING — %.0f of %.0f rpm",
                flywheel.getCurrentRpm(), flywheel.getTargetRpm());
    }

    private String targetName() {
        return limelight.getTargetTagId() == Constants.Vision.BLUE_GOAL_TAG
                ? "BLUE goal (20)" : "RED goal (24)";
    }

    @Override
    public void stop(){
        scheduler.cancelAll();
        drivebase.stop();
        flywheel.stop();
        hood.stop();
        // The scheduler is already cancelled, so push the closed position to the
        // servo ourselves rather than waiting for a periodic() that won't come.
        stopper.stop();
        stopper.periodic();
        turret.stop();
        hardware.limelight.stop();
    }
}
