package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.tuning.HoodTuning;

/**
 * Test 6a — the entire shot, exactly as the match TeleOp runs it, with the
 * drivebase left out so the robot stays put while you read the diagnostics.
 *
 * <p>This is the same aim-range-rev-fire chain as {@code MecanumTeleOp} — same
 * subsystems, same scheduler, same READY gate — with one addition: instead of a
 * single status line, it breaks the gate into its five conditions and shows you
 * which ones are met. In a match "it won't shoot" is one symptom with five
 * causes; here you can see which one it is at a glance.
 *
 * <p>Once this fires reliably at a couple of distances, the robot works, and the
 * only thing left is to prove it with the drivebase under it. Run the real
 * TeleOp for that.
 *
 * <h2>Tuning the shot itself</h2>
 * With the gate passing, what remains is where the ball lands. Shoot from a
 * measured distance, note the RPM and hood position on screen, adjust until it
 * scores, and write that pair into the {@code RANGE_*} tables in
 * {@code Constants.Flywheel} and {@code Constants.Hood}. Do it at three or four
 * distances across your real shooting range; the code interpolates between them
 * and clamps outside, so the near and far ends of the table are the ones that
 * matter most.
 *
 * <p><b>Controls:</b> <b>left bumper</b> rev · <b>left trigger</b> fire ·
 * <b>right bumper</b> intake · <b>A</b> indexer · <b>B</b> reverse indexer ·
 * <b>X</b> blue goal · <b>Y</b> red goal · dpad left/right nudge the hood.
 */
public class ShooterTest extends OpMode {

    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Intake intake;
    private Indexer indexer;
    private Flywheel flywheel;
    private Hood hood;
    private Stopper stopper;
    private Limelight limelight;
    private Turret turret;

    private boolean revving = false;
    private boolean hoodManual = false;
    private boolean firing = false;

    @Override
    public void init() {
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        hardware.initFlywheel(hardwareMap);
        hardware.initTurret(hardwareMap);
        hardware.initHood(hardwareMap);
        hardware.initStopper(hardwareMap);
        hardware.initLimelight(hardwareMap);

        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        flywheel = new Flywheel(hardware);
        hood = new Hood(hardware);
        stopper = new Stopper(hardware);
        limelight = new Limelight(hardware);
        turret = new Turret(hardware, limelight);

        scheduler.reset();
        // Same order as the match: vision refreshes before the turret reads it.
        scheduler.registerSubsystem(intake, indexer, flywheel, hood, stopper, limelight, turret);

        telemetry.addLine("Shooter end-to-end. Same chain as the match TeleOp.");
        telemetry.addLine("LB revs, LT fires. The gate breakdown says what's missing.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.BLUE_GOAL_TAG);
        }
        if (gamepad1.yWasPressed()) {
            limelight.setTargetTagId(Constants.Vision.RED_GOAL_TAG);
        }

        turret.setState(Turret.State.AUTO_AIM);

        double distance = limelight.getDistanceMeters();

        // Hood: auto-ranges like the match, unless you have taken it over to hunt
        // for an angle by hand.
        if (gamepad1.dpadLeftWasPressed()) {
            hoodManual = true;
            HoodTuning.TEST_POSITION = hood.getCommandedPosition() - 0.01;
        }
        if (gamepad1.dpadRightWasPressed()) {
            hoodManual = true;
            HoodTuning.TEST_POSITION = hood.getCommandedPosition() + 0.01;
        }
        if (gamepad1.dpadUpWasPressed()) {
            hoodManual = false;
        }
        if (hoodManual) {
            hood.setPosition(HoodTuning.TEST_POSITION);
        } else if (distance > 0) {
            hood.setForDistance(distance);
        }

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

        boolean ready = turret.isOnTarget() && flywheel.atTargetRpm();
        boolean fire = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // Same burst latch as the match TeleOp: READY arms it, wider bands keep
        // it going, so the gate doesn't slam between every ball.
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
            stopper.open();
            indexer.setState(stopper.isSettled() ? Indexer.State.FEEDING : Indexer.State.IDLE);
        } else {
            stopper.block();
            if (fire) {
                indexer.setState(Indexer.State.IDLE);
            } else if (gamepad1.b) {
                indexer.setState(Indexer.State.REVERSING);
            } else {
                indexer.setState(gamepad1.a ? Indexer.State.FEEDING : Indexer.State.IDLE);
            }
        }

        scheduler.run();

        // --- The gate, broken into its parts ---
        telemetry.addData(">> ", !revving ? "IDLE — press LB to rev"
                : ready ? (fire ? "FIRING" : "READY — hold LT")
                : "REVVING — see the gate below");
        telemetry.addLine();
        telemetry.addLine("--- what the shot is waiting on ---");
        gate("camera sees the goal", limelight.hasTarget());
        gate("distance is valid", distance > 0);
        gate("turret locked on", turret.isOnTarget());
        gate("flywheel at speed", flywheel.atTargetRpm());
        gate("gate finished opening", stopper.isSettled());
        telemetry.addLine();
        telemetry.addData("Distance", distance > 0 ? String.format("%.2f m", distance) : "--");
        telemetry.addData("Flywheel", "%.0f / %.0f rpm",
                flywheel.getCurrentRpm(), flywheel.getTargetRpm());
        telemetry.addData("Hood", "%.3f%s", hood.getCommandedPosition(),
                hoodManual ? "  MANUAL (dpad up = back to auto)" : "  auto-ranged");
        telemetry.addData("Turret command", "%.3f", turret.getCommandedPosition());
        telemetry.addData("Stopper", stopper.getState());
        telemetry.addLine();
        telemetry.addLine("Scored? Write this distance + rpm + hood into the");
        telemetry.addLine("RANGE_ tables in Constants.Flywheel and Constants.Hood.");
        telemetry.update();
    }

    private void gate(String label, boolean ok) {
        telemetry.addData(ok ? "  OK  " : "  no  ", label);
    }

    @Override
    public void stop() {
        scheduler.cancelAll();
        flywheel.stop();
        hood.stop();
        stopper.stop();
        stopper.periodic();
        turret.stop();
        hardware.limelight.stop();
    }
}
