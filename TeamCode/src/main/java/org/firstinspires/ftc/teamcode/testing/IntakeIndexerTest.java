package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Test 2a — intake and indexer, each on its own button.
 *
 * <p>What to prove, with a ball in hand:
 * <ul>
 *   <li><b>INTAKING pulls a ball IN, not out.</b> If it spits, flip
 *       {@code Constants.Intake.DIRECTION}. Do not "fix" it by swapping the
 *       buttons in TeleOp — autonomous calls the same enum and would then run
 *       backwards.</li>
 *   <li><b>FEEDING moves a ball TOWARD the flywheel.</b> Same rule for
 *       {@code Constants.Indexer.DIRECTION}.</li>
 *   <li><b>The current draw settles.</b> Free-running current should be low and
 *       steady. If it climbs and stays high, something is binding. Deliberately
 *       jam a ball for a second and watch the number so you know what a stall
 *       looks like on this robot — that is the reading you will be trying to
 *       recognise later when the shooter mysteriously stops feeding.</li>
 * </ul>
 *
 * <p><b>Controls:</b> <b>right bumper</b> intake in · <b>right trigger</b>
 * intake out · <b>A</b> indexer feed · <b>B</b> indexer reverse. All held, not
 * latched — let go and everything stops.
 */
public class IntakeIndexerTest extends OpMode {

    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Hardware hardware = new Hardware();
    private Intake intake;
    private Indexer indexer;

    @Override
    public void init() {
        hardware.initIntake(hardwareMap);
        hardware.initIndexer(hardwareMap);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        telemetry.addLine("Intake / indexer test. Nothing latches — hold a button.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            intake.setState(Intake.State.OUTTAKING);
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intake.setState(Intake.State.INTAKING);
        } else {
            intake.setState(Intake.State.IDLE);
        }

        if (gamepad1.a) {
            indexer.setState(Indexer.State.FEEDING);
        } else if (gamepad1.b) {
            indexer.setState(Indexer.State.REVERSING);
        } else {
            indexer.setState(Indexer.State.IDLE);
        }

        intake.periodic();
        indexer.periodic();

        telemetry.addData(">> Intake", "%s   (RB in / RT out)", intake.getState());
        telemetry.addData("   power / current", "%+.2f  |  %.2f A",
                hardware.intake.getPower(), hardware.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
        telemetry.addData(">> Indexer", "%s   (A feed / B reverse)", indexer.getState());
        telemetry.addData("   power / current", "%+.2f  |  %.2f A",
                hardware.indexer.getPower(), hardware.indexer.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine();
        telemetry.addLine("INTAKING must pull a ball IN.");
        telemetry.addLine("FEEDING must push it TOWARD the flywheel.");
        telemetry.addLine("Wrong way? Flip DIRECTION in Constants, not the buttons.");
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.setState(Intake.State.IDLE);
        indexer.setState(Indexer.State.IDLE);
        intake.periodic();
        indexer.periodic();
    }
}
