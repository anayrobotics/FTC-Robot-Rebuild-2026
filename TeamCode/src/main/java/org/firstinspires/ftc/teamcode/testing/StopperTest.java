package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.tuning.StopperTuning;

/**
 * Test 2b — find the two positions of the gate that sits between the indexer
 * and the flywheel, and time how long it takes to travel between them.
 *
 * <p>Both positions in {@code Constants.Stopper} are guesses. Nudge them here
 * with a ball actually in the robot, then copy the numbers back into the source
 * — Panels edits live in RAM and are gone at the next restart.
 *
 * <ul>
 *   <li><b>BLOCKING</b> — the smallest movement that reliably holds a ball off
 *       the flywheel. Going further than you need just wastes travel time and
 *       risks stalling the servo against the ball.</li>
 *   <li><b>OPEN</b> — fully clear of the ball path. Watch the ball, not the
 *       servo horn; a gate that only mostly clears will feed fine on the bench
 *       and jam under match-speed feeding.</li>
 *   <li><b>TRAVEL_TIME_S</b> — how long BLOCKING to OPEN actually takes. This is
 *       a plain timer, not feedback, and the shooter waits it out before feeding.
 *       Too short and a ball gets rammed into a half-open gate. Too long and
 *       every shot is delayed by the difference. Flip A and B and count.</li>
 * </ul>
 *
 * <p><b>Controls:</b> <b>A</b> open · <b>B</b> block · bumpers nudge the
 * position of whichever state you are in by 0.01 · dpad up/down nudge
 * TRAVEL_TIME_S by 0.05 s.
 */
public class StopperTest extends OpMode {

    private final Hardware hardware = new Hardware();
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

    private Stopper stopper;

    @Override
    public void init() {
        hardware.initStopper(hardwareMap);
        stopper = new Stopper(hardware);
        telemetry.addLine("Stopper test. A = open, B = block, bumpers nudge.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            stopper.open();
        }
        if (gamepad1.bWasPressed()) {
            stopper.block();
        }

        // Bumpers move whichever position the gate is currently sitting at, so
        // what you see move is always what you are editing.
        double nudge = 0;
        if (gamepad1.rightBumperWasPressed()) {
            nudge = 0.01;
        }
        if (gamepad1.leftBumperWasPressed()) {
            nudge = -0.01;
        }
        if (nudge != 0) {
            if (stopper.isOpen()) {
                StopperTuning.OPEN_POSITION =
                        Range.clip(StopperTuning.OPEN_POSITION + nudge, 0, 1);
            } else {
                StopperTuning.BLOCKING_POSITION =
                        Range.clip(StopperTuning.BLOCKING_POSITION + nudge, 0, 1);
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            StopperTuning.TRAVEL_TIME_S += 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            StopperTuning.TRAVEL_TIME_S = Math.max(0, StopperTuning.TRAVEL_TIME_S - 0.05);
        }

        stopper.periodic();

        telemetry.addData(">> Gate", "%s   (A open / B block)", stopper.getState());
        telemetry.addData(">> Editing", "%s   (bumpers +/- 0.01)",
                stopper.isOpen() ? "OPEN_POSITION" : "BLOCKING_POSITION");
        telemetry.addLine();
        telemetry.addLine("Copy both numbers into Constants.Stopper when done —");
        telemetry.addLine("Panels edits live in RAM and die at the next restart.");

        panels.addData("state", stopper.getState());
        panels.addData("commanded", stopper.getCommandedPosition());
        panels.addData("settled", stopper.isSettled());
        panels.addData("BLOCKING_POSITION", StopperTuning.BLOCKING_POSITION);
        panels.addData("OPEN_POSITION", StopperTuning.OPEN_POSITION);
        panels.addData("TRAVEL_TIME_S", StopperTuning.TRAVEL_TIME_S);
        panels.update(telemetry);
    }

    @Override
    public void stop() {
        stopper.stop();
        stopper.periodic();
    }
}
