package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SetFlywheelRpmCommand;
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
 * <h2>Controls — gamepad1 only</h2>
 * <ul>
 *   <li><b>Left stick</b> — translate. Always field-relative.</li>
 *   <li><b>Right stick X</b> — turn.</li>
 *   <li><b>Options</b> — re-zero field-forward. Point the robot downfield and
 *       press, at the start and any time the heading drifts.</li>
 *   <li><b>Right bumper</b> — toggle the intake on/off.</li>
 *   <li><b>Right trigger</b> — spit out, while held.</li>
 *   <li><b>A</b> — toggle the indexer on/off.</li>
 *   <li><b>B</b> — reverse the indexer while held, to back a jam out.</li>
 *   <li><b>X</b> — spin the flywheel up to the preset. <b>Y</b> — stop it.</li>
 *   <li><b>Dpad left / right</b> — nudge the turret by hand, while held.</li>
 *   <li><b>Left bumper</b> — hold to auto-aim: the turret tracks the goal and
 *       the flywheel auto-ranges to the measured distance.</li>
 *   <li><b>Left trigger</b> — hold to FIRE. The gate only opens and the indexer
 *       only feeds once the turret is locked on AND the flywheel is up to
 *       speed, so a shot that would miss never leaves the robot.</li>
 * </ul>
 *
 * <p>The hood has no buttons — it auto-ranges off the Limelight every loop,
 * whether or not you're holding auto-aim.
 *
 * <p>Both bumper/trigger pairs sit under the left hand for shooting (aim, fire)
 * and the right hand for ball handling (intake, spit), so neither thumb has to
 * leave its stick mid-drive.
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
        telemetry.addLine("RB intake toggle / RT spit  |  A indexer toggle / B reverse");
        telemetry.addLine("X-Y flywheel  |  dpad L-R turret nudge  |  LB aim, LT fire");
        telemetry.addLine("Options re-zeroes heading.");
        telemetry.update();
    }

    @Override
    public void loop(){
        // --- Driving ---
        if (gamepad1.optionsWasPressed()) {
            drivebase.resetHeading();
        }
        drivebase.driveWithGamepad(gamepad1, true);

        // Both shooter holds live under the left hand.
        boolean autoAim = gamepad1.left_bumper;
        boolean fire = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // --- Turret ---
        // Auto-aim wins; failing that, dpad left/right nudges it by hand.
        // Everything released parks the servo — a CRServo left running has no
        // travel limit and would sweep into a hard stop.
        if (autoAim) {
            turret.setState(Turret.State.AUTO_AIM);
        } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
            turret.setState(Turret.State.MANUAL);
            turret.setManualPower(gamepad1.dpad_left
                    ? -Constants.Turret.MANUAL_NUDGE_POWER
                    : Constants.Turret.MANUAL_NUDGE_POWER);
        } else {
            turret.setState(Turret.State.IDLE);
        }

        // --- Hood: fully automatic ---
        // No buttons at all. Any time the Limelight has a range read the hood
        // tracks it; with no read it holds its last angle.
        double distance = limelight.getDistanceMeters();
        if (distance > 0) {
            hood.setForDistance(distance);
        }

        // --- Flywheel ---
        // X/Y are the manual preset and stop. While auto-aim is held the
        // measured distance overrides the target RPM, falling back to the preset
        // when there's no range read yet.
        if (gamepad1.xWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, Constants.Flywheel.SHOOT_RPM));
        }
        if (gamepad1.yWasPressed()) {
            scheduler.schedule(new SetFlywheelRpmCommand(flywheel, 0));
        }
        if (autoAim) {
            flywheel.setTargetRpm(distance > 0
                    ? Flywheel.rpmForDistance(distance)
                    : Constants.Flywheel.SHOOT_RPM);
        }

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
        boolean ready = turret.isOnTarget() && flywheel.atTargetRpm();
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

        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("Intake", "%s%s", intake.getState(), intakeLatched ? " (latched)" : "");
        telemetry.addData("Indexer", "%s%s", indexer.getState(), indexerLatched ? " (latched)" : "");
        telemetry.addData("Flywheel target", flywheel.getTargetRpm());
        telemetry.addData("Flywheel actual", "%.0f rpm", flywheel.getCurrentRpm());
        telemetry.addData("Flywheel at speed", flywheel.atTargetRpm());
        telemetry.addData("Hood position", "%.2f", hood.getCommandedPosition());
        telemetry.addData("Stopper", stopper.getState());
        telemetry.addLine();
        telemetry.addData("Aiming at", targetName());
        telemetry.addData("Turret", turret.getState());
        telemetry.addData("Target visible", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
            telemetry.addData("Distance (m)", "%.2f", distance);
        }
        telemetry.addData("Turret on target", turret.isOnTarget());
        telemetry.addData("READY TO SHOOT", ready);
        telemetry.update();
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
