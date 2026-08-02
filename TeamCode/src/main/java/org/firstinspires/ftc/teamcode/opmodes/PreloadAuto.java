package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SetIndexerStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * The simplest auto that scores: back off the launch line, then dump the
 * preloads at the goal.
 *
 * <p>Three timed phases, no pathing and no odometry — just open-loop drive time
 * and a burst:
 * <ol>
 *   <li><b>BACK_UP</b> — drive straight back for {@link #DRIVE_TIME_S}. The
 *       flywheel is already revving and the turret is already hunting the tag
 *       during this, so the shot is ready the moment we stop.</li>
 *   <li><b>SHOOT</b> — hold still and feed the indexer for
 *       {@link #FEED_TIME_S}, starting as soon as the shot is aimed and up to
 *       speed (or {@link #SPINUP_TIMEOUT_S}, whichever comes first).</li>
 *   <li><b>DONE</b> — everything off, sit still for the rest of auto.</li>
 * </ol>
 *
 * <p>Like {@link AutoAimTeleOp}, this runs the no-hood/no-stopper robot: it uses
 * the per-group {@code hardware.initX()} calls so a missing hood or stopper
 * servo can't throw at init, and nothing holds a ball off the flywheel except
 * the decision below to keep the indexer idle until the wheel is at speed.
 *
 * <h2>Alliance</h2>
 * Picked on the INIT screen — <b>X</b> for blue, <b>B</b> for red, shown in
 * telemetry. This only sets which tag the turret chases; the drive is a straight
 * line back, so it's mirror-symmetric and needs no other change.
 *
 * <h2>Tuning</h2>
 * The four numbers at the top of this class are the whole routine. Start with
 * {@link #DRIVE_TIME_S} — time it on the real field so the robot ends up where
 * you want to shoot from, then confirm the Limelight can still see the goal tag
 * from there.
 */
@Autonomous(name = "Preload Auto", group = "Auto")
public class PreloadAuto extends OpMode {
    // --- The whole routine, in four numbers ---

    // Backwards drive power and how long to hold it. Open loop: no encoder
    // target, no localizer, just power for a time. Negative axial is backwards;
    // flip the sign here if your robot needs to go the other way.
    private static final double DRIVE_POWER = 0.4;
    private static final double DRIVE_TIME_S = 0.6;

    // How long to run the indexer once the burst starts. Sized to empty the
    // preloads, not to shoot for a fixed count — there's no ball sensor.
    private static final double FEED_TIME_S = 2.0;

    // If the shot never reads ready (no tag in view, flywheel short of target),
    // feed anyway after this long. A blind shot at the preset RPM is worth more
    // than sitting still through auto, and with no tag the turret has already
    // returned to neutral — i.e. pointed straight ahead.
    private static final double SPINUP_TIMEOUT_S = 2.0;

    private enum Phase {
        BACK_UP,
        SHOOT,
        DONE
    }

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Limelight limelight;
    private Turret turret;

    private Phase phase = Phase.BACK_UP;
    // Time in the CURRENT phase, reset on every transition.
    private final ElapsedTime phaseTimer = new ElapsedTime();
    // Time since the indexer started feeding. Separate from phaseTimer so a slow
    // spinup eats into SPINUP_TIMEOUT_S and not into the feed window.
    private final ElapsedTime feedTimer = new ElapsedTime();
    private boolean feeding = false;

    // Chosen on the INIT screen with X / B.
    private int goalTagId = Constants.Vision.DEFAULT_TARGET_TAG;

    // Last state actually commanded, so we don't re-schedule the same command
    // fifty times a second.
    private Indexer.State lastIndexerState = Indexer.State.IDLE;

    @Override
    public void init() {
        // Per-group init rather than hardware.init(): the full init pulls the
        // hood and stopper servos out of the Robot Configuration and throws if
        // they aren't there, and neither is on the robot this auto runs on.
        hardware.initDrive(hardwareMap);
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);
        hardware.initTurret(hardwareMap);
        hardware.initLimelight(hardwareMap);
        hardware.initImu(hardwareMap);

        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);

        scheduler.reset();
        // Order matters: the Limelight must refresh BEFORE the Turret reads it,
        // so the turret aims on this loop's fresh vision data.
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
        telemetry.addLine("Back up, then shoot the preloads. No pathing.");
        telemetry.addData("  back up", "%.2f power for %.2f s", DRIVE_POWER, DRIVE_TIME_S);
        telemetry.addData("  then feed", "%.2f s", FEED_TIME_S);
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.setTargetTagId(goalTagId);
        phase = Phase.BACK_UP;
        phaseTimer.reset();
        // Rev and start hunting the tag from the very first loop, so the whole
        // spinup happens while we're driving instead of after we stop.
        flywheel.setTargetRpm(Constants.Flywheel.SHOOT_RPM);
        turret.setState(Turret.State.AUTO_AIM);
    }

    @Override
    public void loop() {
        // Auto-range every loop once a tag is in view, exactly as TeleOp does.
        // With no read yet this holds the preset, which is also the blind-shot
        // value the SPINUP_TIMEOUT_S fallback ends up firing at.
        double distance = limelight.getDistanceMeters();
        double targetRpm = distance > 0
                ? Flywheel.rpmForDistance(distance)
                : Constants.Flywheel.SHOOT_RPM;

        Indexer.State wantIndexer = Indexer.State.IDLE;

        switch (phase) {
            case BACK_UP:
                flywheel.setTargetRpm(targetRpm);
                // Robot-centric on purpose: heading isn't zeroed to anything
                // meaningful at auto start, so "back" means back relative to
                // the robot, which is the only frame we can trust here.
                drivebase.drive(-DRIVE_POWER, 0, 0);
                if (phaseTimer.seconds() >= DRIVE_TIME_S) {
                    drivebase.stop();
                    phase = Phase.SHOOT;
                    phaseTimer.reset();
                }
                break;

            case SHOOT:
                flywheel.setTargetRpm(targetRpm);
                drivebase.stop();

                if (!feeding) {
                    // With no tag in view the turret has gone to neutral and
                    // isOnTarget() is false by definition, so requiring a lock
                    // would mean never shooting. Require it only when there's
                    // actually something to lock onto.
                    boolean aimed = !limelight.hasTarget() || turret.isOnTarget();
                    if ((aimed && flywheel.atTargetRpm())
                            || phaseTimer.seconds() >= SPINUP_TIMEOUT_S) {
                        feeding = true;
                        feedTimer.reset();
                    }
                }

                if (feeding) {
                    // Deliberately NOT re-gated on at-speed each loop. A ball
                    // through the wheel costs a couple hundred RPM, and pausing
                    // the feed to wait that out inside a 2 second window would
                    // cost more shots than it saves. The PID recovers between
                    // balls on its own.
                    wantIndexer = Indexer.State.FEEDING;
                    if (feedTimer.seconds() >= FEED_TIME_S) {
                        phase = Phase.DONE;
                        phaseTimer.reset();
                    }
                }
                break;

            case DONE:
            default:
                drivebase.stop();
                flywheel.stop();
                turret.setState(Turret.State.RETURN_TO_ORIGIN);
                break;
        }

        if (wantIndexer != lastIndexerState) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, wantIndexer));
            lastIndexerState = wantIndexer;
        }

        scheduler.run();

        // scheduler.run() just refreshed vision, so re-read the range rather
        // than printing the value the control loop used above — otherwise the
        // frame a tag first appears reads "visible: true" next to "-1.00".
        double shownDistance = limelight.getDistanceMeters();

        telemetry.addData(">> Phase", "%s   %.2f s", phase, phaseTimer.seconds());
        telemetry.addLine();
        telemetry.addData("Aiming at", targetName());
        telemetry.addData("Target visible", limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("Distance (m)", "%.2f", shownDistance);
        }
        telemetry.addData("Turret", "%s%s", turret.getState(),
                turret.isOnTarget() ? " — LOCKED" : "");
        telemetry.addData("Flywheel", "%.0f / %.0f rpm%s",
                flywheel.getCurrentRpm(), flywheel.getTargetRpm(),
                flywheel.atTargetRpm() ? " — at speed" : "");
        telemetry.addData("Indexer", "%s%s", indexer.getState(),
                feeding ? String.format("   feeding %.2f s", feedTimer.seconds()) : "");
        telemetry.update();
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
        indexer.setState(Indexer.State.IDLE);
        indexer.periodic();
        intake.setState(Intake.State.IDLE);
        intake.periodic();
        turret.stop();
        hardware.limelight.stop();
    }
}
