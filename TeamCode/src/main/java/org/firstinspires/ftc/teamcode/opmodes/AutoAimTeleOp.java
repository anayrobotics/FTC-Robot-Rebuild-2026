package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SetIndexerStateCommand;
import org.firstinspires.ftc.teamcode.commands.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
// import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tuning.TurretTuning;

/**
 * The whole robot on one gamepad — MINUS the hood and the stopper.
 *
 * <p>This is {@link MecanumTeleOp} with the two shooter servos taken out, for
 * driving and tuning auto-aim before the hood and gate are on the robot:
 * <ul>
 *   <li><b>Stopper</b> — gone entirely. There is no gate holding the next ball
 *       back, so the indexer feeds straight into the flywheel and the READY
 *       check is the only thing standing between a ball and the wheel. Nothing
 *       here commands the stopper servo, so if one IS plugged in it just sits
 *       wherever it was left.</li>
 *   <li><b>Hood</b> — commented out, not deleted. Every line it needs is still
 *       here behind {@code //}, marked {@code HOOD:}; un-comment them all
 *       (there are five spots: the import, the field, init, the loop, stop) and
 *       you have hood auto-ranging back. The launch angle is whatever the
 *       hood is mechanically sitting at until then.</li>
 * </ul>
 *
 * <p>Because the hood and stopper are out, this uses the per-group
 * {@code hardware.initX()} calls rather than {@link Hardware#init}. The full
 * init throws if any device is missing from the Robot Configuration, which
 * would stop you driving over a hood servo that isn't wired yet.
 *
 * <h2>Alliance</h2>
 * Picked on the INIT screen — <b>X</b> for blue, <b>B</b> for red, shown in
 * telemetry. Unlike {@link BlueTeleOp}/{@link RedTeleOp} this is one selectable
 * OpMode, because it's a practice config and re-picking beats stopping.
 *
 * <h2>Controls — gamepad1 only, same as the match TeleOp</h2>
 * <ul>
 *   <li><b>Left stick</b> — translate, field-relative. <b>Right stick X</b> — turn.</li>
 *   <li><b>Dpad up</b> — re-zero field-forward.</li>
 *   <li><b>Left bumper</b> — toggle the flywheel rev.</li>
 *   <li><b>Left trigger</b> — hold to FIRE (gated on READY).</li>
 *   <li><b>Right bumper</b> — toggle the intake. <b>Right trigger</b> — spit, while held.</li>
 *   <li><b>A</b> — toggle the indexer. <b>B</b> — reverse it while held, to back a jam out.</li>
 *   <li><b>Dpad left / right</b> — nudge the turret by hand while held; resumes
 *       hunting the goal on release.</li>
 *   <li><b>Dpad down</b> — park the turret at its origin and hold it there. You
 *       can't shoot while parked. Press again to resume tracking.</li>
 * </ul>
 */
@TeleOp(name = "TeleOp — Auto-Aim (no hood/stopper)", group = "Drive")
public class AutoAimTeleOp extends OpMode {
    // Trigger past this counts as "held".
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Limelight limelight;
    private Turret turret;
    // HOOD: un-comment to put the hood back in.
    // private Hood hood;

    // Chosen on the INIT screen with X / B.
    private int goalTagId = Constants.Vision.DEFAULT_TARGET_TAG;

    // Latched by the left bumper: is the shooter spun up and tracking range?
    private boolean revving = false;
    // Latched by dpad down: hold the turret at its origin instead of tracking.
    private boolean turretParked = false;
    // Latched by the right bumper. The spit-out trigger outranks it while held
    // without clearing it, so releasing the trigger resumes intaking.
    private boolean intakeLatched = false;
    // Latched by A, same idea — outranked by the fire trigger and by B.
    private boolean indexerLatched = false;
    // A burst in progress. Armed by READY, sustained on wider bands, dropped
    // when the trigger is released. See the loop for why the two differ.
    private boolean firing = false;

    // Last state actually commanded, so we don't re-schedule the same command
    // fifty times a second.
    private Intake.State lastIntakeState = Intake.State.IDLE;
    private Indexer.State lastIndexerState = Indexer.State.IDLE;

    @Override
    public void init() {
        // Per-group init, not hardware.init(): the full init pulls the hood and
        // stopper servos out of the Robot Configuration and throws if they
        // aren't there, which is the whole thing this OpMode exists to avoid.
        hardware.initDrive(hardwareMap);
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);
        hardware.initTurret(hardwareMap);
        hardware.initLimelight(hardwareMap);
        hardware.initImu(hardwareMap);
        // HOOD: un-comment for the hood servo.
        // hardware.initHood(hardwareMap);
        // Deliberately no initStopper() — no gate on this robot.

        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);
        // HOOD: un-comment.
        // hood = new Hood(hardware);

        scheduler.reset();
        // Order matters: the Limelight must refresh BEFORE the Turret reads it,
        // so the turret aims on this loop's fresh vision data.
        // HOOD: add `hood` to this list when you bring it back.
        scheduler.registerSubsystem(intake, indexer, flywheel, limelight, turret);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            goalTagId = Constants.Vision.BLUE_GOAL_TAG;
        }
        if (gamepad1.bWasPressed()) {
            goalTagId = Constants.Vision.RED_GOAL_TAG;
        }

        telemetry.addData(">> Aiming at", "%s   (X blue / B red)", targetName());
        telemetry.addLine();
        telemetry.addLine("No hood, no stopper — nothing holds a ball off the wheel,");
        telemetry.addLine("so the indexer only feeds once the shot reads READY.");
        telemetry.addLine();
        telemetry.addLine("LB revs, LT fires  |  RB intake / RT spit  |  A indexer / B reverse");
        telemetry.addLine("Dpad up re-zeroes heading  |  dpad L-R nudges turret  |  dpad down parks");
        telemetry.update();
    }

    @Override
    public void start() {
        // Locked in here rather than in the loop, so a stray B press mid-match
        // (which reverses the indexer) can't swing the turret at the other
        // alliance's goal.
        limelight.setTargetTagId(goalTagId);
    }

    @Override
    public void loop() {
        // --- Driving ---
        if (gamepad1.dpadUpWasPressed()) {
            drivebase.resetHeading();
        }
        drivebase.driveWithGamepad(gamepad1, true);

        // --- Turret: always hunting the goal ---
        // Parking has to LATCH rather than run while held: auto-aim is the
        // default every loop, so a momentary park would be dragged straight back
        // to the goal the loop after you let go. Dpad down again — or any manual
        // nudge — releases it back to tracking.
        if (gamepad1.dpadDownWasPressed()) {
            turretParked = !turretParked;
        }
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            turretParked = false;
            turret.setState(Turret.State.MANUAL);
            turret.setManualPower(gamepad1.dpad_left ? -1.0 : 1.0);
        } else if (turretParked) {
            turret.setState(Turret.State.RETURN_TO_ORIGIN);
        } else {
            turret.setState(Turret.State.AUTO_AIM);
        }

        // Range to the goal, or -1 with no target. Drives the flywheel table
        // below (and the hood, when it's back).
        double distance = limelight.getDistanceMeters();

        // HOOD: --- Hood: always ranging ---
        // Any time the Limelight has a range read the hood tracks it; with no
        // read it holds its last angle.
        // if (distance > 0) {
        //     hood.setForDistance(distance);
        // }

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
        //
        // It matters more here than it does with a gate fitted. In the match
        // TeleOp the stopper is the thing physically keeping a ball off the
        // wheel and this flag only decides when to open it; with no stopper this
        // flag IS the gate, and a ball fed early goes into a wheel that hasn't
        // spun up.
        boolean ready = turret.isOnTarget() && flywheel.atTargetRpm();
        boolean fire = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // Starting a burst and continuing one are different questions, and
        // asking the strict one twice is what makes a shooter feel broken.
        // READY is a 25 RPM window and a 1 degree lock, but a ball through the
        // wheel costs a couple of hundred RPM — re-check READY every loop and
        // the feed stutters between every single shot. So: READY arms the burst,
        // and much wider bands sustain it. The shot still stops the instant the
        // aim genuinely goes (target lost, turret parked, wheel really bogged).
        if (!fire) {
            firing = false;
        } else if (ready) {
            firing = true;
        }
        boolean keepFiring = firing
                && turret.getAimErrorDeg() <= Constants.Turret.KEEP_AIM_TOLERANCE_DEG
                && flywheel.atTargetRpm(Constants.Flywheel.RPM_KEEP_TOLERANCE);

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

        // --- Indexer ---
        // Three claims on one motor, highest priority first: the fire trigger,
        // a held B to back a jam out, then the A toggle for loading up.
        //
        // With the gate fitted this branch would open the stopper and wait out
        // its travel time before feeding. There is no gate, so a confirmed burst
        // feeds immediately — one less thing between READY and a ball leaving.
        if (gamepad1.aWasPressed()) {
            indexerLatched = !indexerLatched;
        }
        Indexer.State wantIndexer;
        if (keepFiring) {
            wantIndexer = Indexer.State.FEEDING;
        } else if (fire) {
            // Trigger held but not confirmed: hold the ball back rather than
            // letting a shot go that would miss.
            wantIndexer = Indexer.State.IDLE;
        } else if (gamepad1.b) {
            wantIndexer = Indexer.State.REVERSING;
        } else {
            wantIndexer = indexerLatched ? Indexer.State.FEEDING : Indexer.State.IDLE;
        }
        if (wantIndexer != lastIndexerState) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, wantIndexer));
            lastIndexerState = wantIndexer;
        }

        scheduler.run();

        // scheduler.run() just refreshed vision, so re-read the range rather
        // than printing the value the control loop used above. Otherwise, on the
        // frame a tag first appears, telemetry reads "Target visible: true" next
        // to "Distance: -1.00" and sends you hunting a camera bug that isn't one.
        double shownDistance = limelight.getDistanceMeters();

        // Shooter state first and loudest — this line is what the driver is
        // actually watching, and it says what to do next.
        telemetry.addLine(shooterStatus(ready, fire, keepFiring));
        telemetry.addLine();
        telemetry.addData("Aiming at", targetName());
        telemetry.addData("Target visible", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
            telemetry.addData("Distance (m)", "%.2f", shownDistance);
        }
        telemetry.addData("Turret", "%s%s", turret.getState(),
                turret.isOnTarget() ? " — LOCKED" : "");
        telemetry.addData("Turret command", "%.3f   neutral %.3f",
                turret.getCommandedPosition(), TurretTuning.NEUTRAL_POSITION);
        telemetry.addData("Flywheel", "%.0f / %.0f rpm%s",
                flywheel.getCurrentRpm(), flywheel.getTargetRpm(),
                flywheel.atTargetRpm() ? " — at speed" : "");
        // HOOD: un-comment.
        // telemetry.addData("Hood position", "%.2f", hood.getCommandedPosition());
        telemetry.addLine();
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("Intake", "%s%s", intake.getState(), intakeLatched ? " (latched)" : "");
        telemetry.addData("Indexer", "%s%s", indexer.getState(), indexerLatched ? " (latched)" : "");
        telemetry.addLine("Hood + stopper are OUT of this OpMode.");
        telemetry.update();
    }

    // One line telling the driver where the shot is and what to press. Ordered
    // by what's blocking the shot, most fundamental first.
    private String shooterStatus(boolean ready, boolean fire, boolean keepFiring) {
        if (!revving) {
            return ">> IDLE — press LB to rev up";
        }
        // Check the burst first. Mid-burst the strict READY flag dips on every
        // ball, and falling through to "aiming..." while balls are visibly
        // leaving the robot reads as a fault when nothing is wrong.
        if (keepFiring) {
            return ">> FIRING";
        }
        if (ready) {
            return fire ? ">> FIRING" : ">> READY TO SHOOT — hold LT to fire";
        }
        if (turretParked) {
            // Parked can't shoot: isOnTarget() is false outside AUTO_AIM. Say so
            // rather than leaving the driver waiting on a lock that isn't coming.
            return ">> REVVING — turret PARKED, dpad down to resume aiming";
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
        return goalTagId == Constants.Vision.BLUE_GOAL_TAG ? "BLUE goal (20)" : "RED goal (24)";
    }

    @Override
    public void stop() {
        scheduler.cancelAll();
        drivebase.stop();
        // The scheduler is already cancelled, so push each safe state to the
        // hardware ourselves — stop() only sets a field, and the periodic()
        // that would normally write it is never going to run.
        flywheel.stop();
        flywheel.periodic();
        turret.stop();
        // HOOD: un-comment.
        // hood.stop();
        // hood.periodic();
        hardware.limelight.stop();
    }
}
