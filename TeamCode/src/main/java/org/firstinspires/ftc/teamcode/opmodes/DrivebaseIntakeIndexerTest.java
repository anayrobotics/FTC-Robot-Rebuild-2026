package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SetIndexerStateCommand;
import org.firstinspires.ftc.teamcode.commands.SetIntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Integration bring-up test for the drivebase together with the intake and
 * indexing subsystems — the pieces that share the robot but nothing to do with
 * the shooter. It runs the full four-wheel mecanum FIELD-RELATIVE drive off the
 * navX2-Micro gyro, plus the single-motor intake and single-motor indexer,
 * driven through the real {@link CommandScheduler} exactly as {@code
 * MecanumTeleOp} wires them.
 *
 * <p>Uses {@link Hardware#initDriveIntakeIndexer} so it runs on a Control Hub
 * configured with only the four drive motors, the navX, the intake motor and
 * the indexer motor (no flywheel / turret / hood / Limelight / hub IMU).
 *
 * <h2>Controls — single driver, all on gamepad1</h2>
 * <ul>
 *   <li><b>Left stick</b> — translate (field-relative when field-centric is on).</li>
 *   <li><b>Right stick X</b> — rotate.</li>
 *   <li><b>Options</b> — re-zero field-forward heading. Point the robot
 *       downfield and press at the start so field-centric forward matches the
 *       field.</li>
 *   <li><b>Back</b> — toggle field-centric / robot-centric.</li>
 *   <li><b>Right bumper</b> — intake IN, <b>Left bumper</b> — intake OUT,
 *       <b>A</b> — intake IDLE.</li>
 *   <li><b>Dpad up</b> — indexer FEED (both motors), <b>Dpad down</b> — indexer
 *       REVERSE (both motors), <b>B</b> — indexer IDLE.</li>
 *   <li><b>X</b> — run the FRONT indexer motor alone, <b>Y</b> — run the BACK
 *       indexer motor alone. Bring-up checks; press B to stop.</li>
 * </ul>
 *
 * <p>FIELD-CENTRIC CHECK: with field-centric on, hold the left stick forward
 * and rotate the robot in place — it should keep driving the same direction
 * across the floor. If forward drives the wrong way as you rotate, flip {@link
 * org.firstinspires.ftc.teamcode.localization.NavXIMU#INVERT}.
 *
 * <p>INDEXER CHECK: the indexer has two motors that must move a ball the SAME
 * way. Run X and then Y and watch each motor on its own — both should push a
 * ball toward the shooter. If one runs backwards, flip that motor's direction
 * ({@code FRONT_DIRECTION} / {@code BACK_DIRECTION} in {@link
 * org.firstinspires.ftc.teamcode.Constants.Indexer}) rather than negating its
 * power, so FEEDING stays correct. Only once both agree should you use dpad up.
 */
@TeleOp(name = "Drivebase + Intake + Indexer Test", group = "Test")
public class DrivebaseIntakeIndexerTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private Drivebase drivebase;
    private Intake intake;
    private Indexer indexer;

    private boolean fieldCentric = true;

    @Override
    public void init() {
        // Only the four drive motors, the navX, and the intake + indexer motors.
        hardware.initDriveIntakeIndexer(hardwareMap);
        drivebase = new Drivebase(hardware);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);

        scheduler.reset();
        scheduler.registerSubsystem(intake, indexer);

        telemetry.addLine("Drivebase + intake + indexer test ready (navX heading).");
        telemetry.addLine("Single driver, gamepad1:");
        telemetry.addLine("  left stick drive, right stick X turn, Back toggle centric, Options zero heading.");
        telemetry.addLine("  RB/LB/A intake in/out/idle, dpad up/down/B indexer feed/reverse/idle.");
        telemetry.addLine("  X/Y run front/back indexer motor alone (direction check).");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Driving ---
        if (gamepad1.backWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.options) {
            drivebase.resetHeading();
        }
        drivebase.driveWithGamepad(gamepad1, fieldCentric);

        // --- Intake ---
        if (gamepad1.rightBumperWasPressed()) {
            scheduler.schedule(new SetIntakeStateCommand(intake, Intake.State.INTAKING));
        }
        if (gamepad1.leftBumperWasPressed()) {
            scheduler.schedule(new SetIntakeStateCommand(intake, Intake.State.OUTTAKING));
        }
        if (gamepad1.aWasPressed()) {
            scheduler.schedule(new SetIntakeStateCommand(intake, Intake.State.IDLE));
        }

        // --- Indexer: both motors together ---
        if (gamepad1.dpadUpWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.FEEDING));
        }
        if (gamepad1.dpadDownWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.REVERSING));
        }
        if (gamepad1.bWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.IDLE));
        }

        // --- Indexer: one motor at a time, to check wiring and direction ---
        if (gamepad1.xWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.FRONT_ONLY));
        }
        if (gamepad1.yWasPressed()) {
            scheduler.schedule(new SetIndexerStateCommand(indexer, Indexer.State.BACK_ONLY));
        }

        // Runs each subsystem's periodic() and services scheduled commands.
        scheduler.run();

        telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(drivebase.getHeading()));
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Indexer", indexer.getState());
        // Both motor powers, so you can see the two stages agree when FEEDING
        // and see exactly which one is moving during the X / Y checks.
        telemetry.addData("  front power", "%.2f", hardware.frontIndexer.getPower());
        telemetry.addData("  back power", "%.2f", hardware.backIndexer.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        scheduler.cancelAll();
        drivebase.stop();
    }
}
